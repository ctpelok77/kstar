Bootstrap: docker
From: ctpelok77/kstar:latest

%setup
    # Just for diagnosis purposes
    hostname -f > $SINGULARITY_ROOTFS/etc/build_host
%runscript
    # This will be called whenever the Singularity container is invoked
    python3 /workspace/kstar/fast-downward.py --build release64 $@

%post

%labels
Name        K*
Description The K* planner
Authors     Michael Katz <michael.katz1@ibm.com>, Shirin Sohrabi <ssohrab@us.ibm.com>, Octavian Udrea <udrea@us.ibm.com> and Dominik Winterer <dominik_winterer@gmx.de>
