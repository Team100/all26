# Wildhat AI Infrastructure

## Overview

This document proposes the cloud infrastructure for running the Wildhat AI
system described in [README.md](README.md) on GCP. The system is idle for weeks
at a time, then busy when several people launch jobs at once, so everything here
scales to zero and provisions capacity on demand. Compute can genuinely cost
nothing while idle; stored bytes cannot. The two real risks to the budget are
accumulated storage and a forgotten GPU job. Everything runs in GCP's
`us-central1`, the cheapest region with L4 GPUs and every service below.

| Need | GCP Service |
|---|---|
| Simulation, training, evals, post-processing | Cloud Run Jobs |
| `runs` and `training` databases | Firestore |
| Logged data, checkpoints, robot uploads | Cloud Storage |
| Visualizer app | Cloud Run service |
| Sign-in | Firebase Authentication |
| Container images | Artifact Registry |

## Compute

Simulation runs, training jobs, evals and post-processing run as Cloud Run Jobs,
which scale to zero, bill per second, start in seconds rather than minutes, and
support one NVIDIA L4 GPU per instance — enough to render simulated camera
images and train the model sizes this project targets. One job definition runs
many tasks at once, which maps onto episodes: one job per run, one task
per episode, so fifty episodes finish in roughly the wall-clock time of one.

## Storage and Databases

All generated artifacts live in one Cloud Storage bucket, keyed by run and
episode so a path is derivable from a `runs` record: logged sensor data,
ground-truth annotations, compiled videos, checkpoints and robot uploads.
Standard storage costs $0.020/GB/month and Autoclass moves untouched data down
to $0.004/GB/month on its own. The robot uploads straight to the bucket with a
signed URL, so no service has to be running to receive it.

The `runs` and `training` databases are Firestore collections, which cost
nothing while idle and whose document model fits a run containing episodes.
Firestore holds metadata and references only; per-episode signals are Parquet
files in the bucket, read directly by the visualizer and queryable in place by
BigQuery if cross-run analytics are ever wanted. Cloud SQL is not used, because
its smallest instance runs continuously at roughly $10/month and stopping it
makes the visualizer unavailable.

## Visualizer

The visualizer is one Cloud Run service with a minimum instance count of zero,
serving both the browser application and the API that reads Firestore and Cloud
Storage. Cold start is one to two seconds. Sign-in is Firebase Authentication
with Google accounts, checked against an allowlist of team accounts.
Identity-Aware Proxy would be more conventional but needs a load balancer
billing roughly $20/month merely to exist, which would be the largest idle cost
in the system.

Recorded `runs` are served from the cloud; live streams are not. Cloud Run
instances are independent, so a `Logger` publishing to one and a browser
subscribing to another never connect. At the bench and in the pit they share a
network anyway, so the visualizer connects to the `Logger` directly over the
LAN, which is less infrastructure, lower latency, and works when the venue
network does not.

## Cost

An idle month costs about $10, nearly all of it storage for roughly 500 GB of
artifacts. An active month in build season, with about twenty L4-hours, costs
$30-60, of which compute is the smaller part. A new billing account includes
$300 in trial credits, and a registered 501(c)(3) can get Google Cloud credits
on the order of $2,000/year through Google for Nonprofits.

Data transfer out to the internet costs $0.12/GB and scrubbing video is exactly
that, so the `PostProcessor` produces low-bitrate proxy videos, the visualizer
scrubs those, and full-resolution video is fetched only on request.

The real financial risk is a GPU job left running over a weekend, so these are
set up before the first job runs:

- A billing budget with alerts, and a Cloud Function that cancels running jobs.
- A project GPU quota of one or two L4s.
- An explicit timeout on every job definition, set per job and justified.
- Labels recording who launched each job, with billing exported to BigQuery.
- An Artifact Registry cleanup policy for untagged images.

## Build Order

1. One bucket, Firestore, a CPU-only Cloud Run Job, the visualizer service,
   Firebase Authentication, and the billing budget — the system end to end.
2. Attach an L4 GPU to the simulation and training jobs.
3. Enable task parallelism so a run's episodes execute concurrently.
4. Add BigQuery when Parquet files cannot answer a cross-run question.

All of it lives in Terraform in this repository. Team membership turns over
every year, and infrastructure that exists only in someone's console history
does not survive graduation.
