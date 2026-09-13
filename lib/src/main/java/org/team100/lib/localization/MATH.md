# Using Uncertainty

Math related to localization.

### Background

In 2026, our alignment requirements are tighter than in previous
years, so we need to pay more attention to localization accuracy.

Previously, we made several simplifying choices:

* A fixed estimate for state variance.
* The variance of vision-derived XY position depended on tag distance.
* The variance of vision-derived rotation was infinite.
* Odometry was assumed to be perfect, i.e. did not change state variance.
* The gyro was assumed to be perfect: other rotation sources were ignored.
* The gyro could be reset manually.

In 2026, most of these choices are replaced:

* State variance evolves as it is updated.
* Vision-derived XY position variance still depends on tag distance.
* Vision-derived rotation variance depends on off-axis angle.
* Odometry and odometry variances add to state variance.
* Gyro variance is constant.
* Odometry variance is dependent on speed.
* The only absolute rotation input is now vision, not the gyro.
* Therefore the gyro cannot be reset: show a tag to a camera to set the rotation.

So there are two types of updates: "between" updates,
which describe the difference between the previous state and the next one,
and "fusion" updates, which are independent estimates of the state itself.

Each "between" update is itself a fusion of two independent estimates,
one from odometry, and one from the gyro.

To apply the "between" update to the state,  we assume that update
measurement and state are independent, so the means add, and the variances add,
with no covariance term.
The assumption of independence comes from the assumption of independence
of each odometry update.  In reality, the main odometry error is probably
bias rather than noise, but we ignore that.

Each "fusion" update computes the resulting state mean and variance using
inverse-variance weighting, or, equivalently, covariance intersection
with equal weights.

To fuse the gyro and odometry, the gyro variance is a constant, larger than
the drift rate, and the odometry variance is quadratic with respect to speed.

To fuse the vision input, variances are drawn from the Wang paper,
and the state variance is whatever has accumulated.

There are many ways to fuse measurements.

* Mixture model.  If the measurements represent different subpopulations,
the "fusion" of the measurements is a weighted average.  Here we use the
formulation called "covariance intersection" using equal weights.  The variance of
the result is never less than the smaller variance of the components. This fusion method
is not really appropriate -- our measurements are not derived from subpopulations --
but it has the advantage of respecting the dispersion of the means, and of not
being overconfident.  It also has the significant flaw of respecting
large variances: fusing a large variance doesn't indicate that the component
has little knowledge of the mean, it indicates that the component variance
is, in fact, large, and should be included.  So this is the wrong choice.
* Weighted mean.  If the measurements represent independent estimates of
the same quantity, the "fusion" is also a weighted average.  The weights that
minimize the resulting variance are the inverse variances of the components.
The mean is the same as above (inverse variance weighting), but the variance
is computed differently.  Because the estimates are independent, there is
no covariance term, and the variance is simply the reciprocal of the sum
of the reciprocals of component variances.  Bayesian updating is the same
as inverse-variance weighted averaging.  This method is also not appropriate
because it ignores mean dispersion, and tends to become overconfident.  
Why?  Because the measurement and the state are not actually independent.

So neither of these is really right. what we really want is a fusion method
that includes the mean dispersion term, and also reduces variance when the
means are similar.

This problem is known: the inverse variance weighting becomes overconfident,
but the mixing model doesn't become confident enough.  The idea of reducing
overconfidence is called "covariance inflation".  As far as I can tell, this
idea isn't a fancy statistics theory, it's just a heuristic.  For example,
you can just impose a minimum state variance (i.e. "adding noise"),
or you can just nudge the state variance towards the update variance,
using fixed or variable weights.

Another common approach to covariance inflation is to use
the update "innovation" (i.e. difference in means) to adjust the inflation,
perhaps with a threshold (this represents a change in state).  This is
similar to the mixture model's approach of simply including the mean
dispersion.  One issue with this approach is *overresponding* to the mean
dispersion: repeated update means will be distributed according to the measurement
variance, resulting in a state variance that mirrors the measurement variance,
even if the measurement variance itself is ignored.  Another issue is that
the overconfidence doesn't disappear if the means are close.

So a combination of approaches might be good: added noise to keep the state
from becoming unresponsive to small innovation, and mean dispersion to adapt to
large innovation.

This combination, using a minimum noise constraint and a small weight on the mean
dispersion of updates, is what is implemented in the `Covariance Inflation` method in
`VariableR1`.

### Gyro model

The gyro can be modeled with two random variables:

* **Bias**, radians/sec.  This is the "drift rate," a slow random walk.
In reality, it is temperature-dependent, but we ignore that.
* **Noise**, radians.  This is the integral of the (Gaussian) sensor noise,
which turns out to also be Gaussian.  

Be use odometry to learn the bias: the drift
rate is the difference between the gyro dtheta and the odometry dtheta for each
update.  Sometimes the odometry isn't very accurate (e.g. when driving fast), and
so has essentially no influence on the drift rate.  When the odometry is very
accurate (e.g. when stopped), it has very firm control over the drift rate.

The noise term has fixed width.  The effect
of gyro noise is simply to inform the variance used in fusion.

As an example, the Kalibr docs mention the ADIS16448 (a MEMS device similar to the
gyro we use) with these noise parameters:

```
white_noise_density = 0.0004 // rad/s/sqrt(hz)
bias_noise_density = 0.000004 // rad/s^2/sqrt(hz)
```
(We should attempt to verify these parameters.)

To understand these parameters, it's useful to know how the MEMS gyroscope works.
It senses the coriolis effect (force in one direction produces movement at 90
degrees in a rotating reference frame) directly, and so the sensor noise
appears in the _rotation rate_ (rad/s).

The white noise variance is inversely proportional to the sample period, i.e. it
indicates that the sensor is simply averaging the sensor noise during the sample period.

So the rate noise density is expressed in rad/s/sqrt(hz).

The gyro integrates this rate to obtain the yaw angle.
Variances add,  so the variance of the yaw is proportional to
the duration of the integral, i.e. it is just the rate noise (rad/s) times the
integral duration (s), yielding yaw noise (rad).  The gyros we typically use
do this integration a bit faster (100hz) than the robot clock (50hz).

We take the difference of two yaw measurements, which is the same as just doing
the integral with different bounds, so again the integrated noise in the
yaw increment is the rate noise times the clock period.

So to compute the noise in a yaw step:

```
gyro_rate = 100 // gyro internal clock, hz
dt = 0.02 // robot main loop period, sec

noise_rate_stddev  = white_noise_density * sqrt(gyro_rate) // rad/s
noise_sample_stddev = noise_rate_stddev * dt               // rad
```

In addition to sensor noise in the rate measurement, gyros also produce
_bias_, i.e. a nonzero rate measurement for zero ground-truth, and this
bias itself changes over time.  There are several additional sources of
noise (see references) but they operate on longer timescales than
we care about.  For our purposes, we can use a single bias noise
term, "bias instability," used as the floor for the bias variance
estimate, which doesn't depend on anything.

The effect should be for the bias variance to converge to a stable estimate and
then change quite slowly, because new evidence doesn't make much difference.

We use several steps to ingest the gyro measurement:

1. Find the gyro increment: the difference between the gyro measurement at the current instant
and the previous step.  The variance in this measurement is a constant, determined by the gyro white noise.
2. Find the odometry rotation increment.  The variance in this measurement depends on drive speed.
3. Subtract the odometry increment from the gyro increment, noting that the variances add.
This is the drift measurement.
4. Fuse this drift estimate with the state drift estimate (and its variance).
This fusion should use the same covariance inflation method mentioned above.
5. Subtract this new state drift estimate from the gyro increment to find the corrected gyro
increment.  Note, the variances add.
6. Fuse the corrected gyro increment with the odometry rotation increment, again using the covariance
inflation method.

The way that GTSAM handles the bias is
[different](https://github.com/borglab/gtsam/blob/develop/examples/ImuFactorsExample.cpp) ...
each time-step uses a fixed noise model, instead of allowing the bias noise
to float, as above.  I think we'll do much the same thing with the covariance
inflation idea, keeping the bias variance from getting too low.

Simple GTSAM examples just uses a *constant* bias term, I guess for short-duration
cases?

### Mixture model

The mean of each fused result is as follows, straightforward weighting by
inverse variance:

```math
\mu = w_A \mu_A + w_B \mu_B
```

The variance of each fused result is a function of the component variances,
with a covariance term from the difference in means:

```math
\sigma^2=w_A\sigma_A^2+w_B\sigma_B^2+w_A w_B(\mu_A-\mu_B)^2
```

The weights are determined using inverse-variance weighting.

```math
w_A = \frac{1}{\sigma_A^2}\left(  \frac{1}{\sigma_A^2} + \frac{1}{\sigma_B^2} \right)^{-1}
```
and
```math
w_B = \frac{1}{\sigma_B^2}\left(  \frac{1}{\sigma_A^2} + \frac{1}{\sigma_B^2} \right)^{-1}

```

Using inverse-variance weighting, the variance can be expressed as the
_harmonic mean_ of the component variances, added to the mean dispersion term.

Note: the gyro produces both an estimate of position, and an estimate of
velocity.  The robot state also contains both postion and velocity.  We could
separate these two dimensions, so that the gyro velocity would update the
state velocity independently of the position update, but we're not doing that.
The gyro position is a simple integral of the velocity, and the state velocity
is a simple difference of the position, so we just ignore the gyro velocity.


References:

* A useful question and describing the covariance due to dispersion of the mean [Stack Overflow](https://stats.stackexchange.com/questions/16608/what-is-the-variance-of-the-weighted-mixture-of-two-gaussians).
* The [sway](https://github.com/Team100/all24/blob/main/studies/sway/src/main/java/org/team100/lib/sway/fusion/LinearPooling.java#L149) project from 2023, which was an unused attempt to make all observations into random variables.
* A [survey of pooling methods](https://arxiv.org/pdf/2202.11633)
* Wikipedia on [inverse variance weighting](https://en.wikipedia.org/wiki/Inverse-variance_weighting).
* Wikipedia on [covariance intersection](https://en.wikipedia.org/wiki/Covariance_intersection), which is the same, if the weights are chosen to be equal.
* [Bayesian update](https://stats.stackexchange.com/questions/237037/bayesian-updating-with-new-data)
* Some slides about [Covariance inflation](https://web.cels.anl.gov/~aattia/Files/Slides/SIAM_MPE_18/mpe18_adaptive.pdf)
* [Gyro model discussion](https://github.com/ethz-asl/kalibr/issues/354#issuecomment-979934812) in the context of the [ETH Kalibr camera/IMU calibration toolkit](https://github.com/ethz-asl/kalibr/wiki).  See also [python example](https://github.com/Team100/all24/blob/main/studies/factor_graph/kalibr_gyro.py) of the gyro model, and a more [complete simulation](https://github.com/Team100/all24/blob/main/studies/factor_graph/gyro_sim.py) from the 2024 GTSAM effort. (Note these contain errors.)
* [Kalibr IMU noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model).  Note the sentence, "From our experience, for lowest-cost sensors, increasing the noise model parameters by a factor of 10x or more may be necessary."
* [more about IMU specs](https://stechschulte.net/2023/10/11/imu-specs.html)
* [example IMU calibration](https://github.com/rpng/ar_table_dataset/blob/master/calibration/kalibr_color_0_imu/d455_calib_02-imu.yaml) with noise = 0.008 (high!) and bias = 1e-5.  More examples can be found on Github by searching for Kalibr configuration YAML files, with the keys gyroscope_noise_density and gyroscope_random_walk.
* [Vectornav examples](https://www.vectornav.com/resources/inertial-navigation-primer/examples/noise)
* [how MEMS sensors work](https://www.vectornav.com/resources/inertial-navigation-primer/theory-of-operation/theory-mems)
* [Coriolis force](https://en.wikipedia.org/wiki/Coriolis_force)
* [Analog Devices gyros](https://www.analog.com/media/en/news-marketing-collateral/solutions-bulletins-brochures/MEMS-IMU-Brochure.pdf)
* [Overview of gyro errors](https://home.engineering.iastate.edu/shermanp/AERE432/lectures/Rate%20Gyros/14-xvagne04.pdf)