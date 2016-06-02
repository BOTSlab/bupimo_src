This package holds commonly useful bits of Python code.  For this code to be
usable in other packages setup.py was required.  It is a bit strange that
this directory contains another called 'bupimo_utils' but that is just the way
it had to work in order to have straightforward import statements in other
packages such as this:

from bupimo_utils.pucks_and_clusters import get_closest_puck

Andrew Vardy
30 May, 2016
