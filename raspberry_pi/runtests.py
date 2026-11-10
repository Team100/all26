import faulthandler
import unittest

loader = unittest.TestLoader()
suite = loader.discover("tests", pattern="*test.py")

# Print a stack trace if the test runner hangs.
# This can be because of zombie threads that should be
# killed cleanly.
faulthandler.enable()
faulthandler.dump_traceback_later(10, repeat=True)

runner = unittest.TextTestRunner(verbosity=2)
runner.run(suite)
