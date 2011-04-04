import os
import sys
import time
import signal
import subprocess
import resource

from processgroup import ProcessGroup


def kill_pgrp(pgrp, sig):
    try:
        os.killpg(pgrp, sig)
    except OSError:
        #TODO: log error somewhere
        pass


def set_limit(kind, amount):
    try:
        resource.setrlimit(kind, (amount, amount))
    except (OSError, ValueError), e:
        #TODO: log error somewhere or return a success flag that makes the call
        # abort
        pass


class Call(subprocess.Popen):
    def __init__(self, args, time_limit=1800, wall_clock_time_limit=1800,
                 mem_limit=2048, kill_delay=5, check_interval=0.1, **kwargs):
        """
        mem_limit =         Memory in MiB
        kill_delay =        How long we wait between SIGTERM and SIGKILL
        check_interval =    How often we query the process group status
        """
        self.time_limit = time_limit
        self.wall_clock_time_limit = wall_clock_time_limit
        self.mem_limit = mem_limit

        self.kill_delay = kill_delay
        self.check_interval = check_interval

        self.log_interval = 5

        stdin = kwargs.get('stdin')
        if type(stdin) is str:
            kwargs['stdin'] = open(stdin)

        for stream_name in ['stdout', 'stderr']:
            stream = kwargs.get(stream_name)
            if type(stream) is str:
                kwargs[stream_name] = open(stream, 'w')

        def prepare_call():
            os.setpgrp()
            set_limit(resource.RLIMIT_CPU, self.time_limit)
            # Memory in Bytes
            set_limit(resource.RLIMIT_AS, self.mem_limit * 1024 * 1024)
            set_limit(resource.RLIMIT_CORE, 0)

        subprocess.Popen.__init__(self, args, preexec_fn=prepare_call, **kwargs)

    def wait(self):
        """Wait for child process to terminate.

        If the process' processgroup exceeds any limit it is killed.
        Returns returncode attribute.
        """
        term_attempted = False
        real_time = 0
        last_log_time = 0
        while True:
            time.sleep(self.check_interval)
            real_time += self.check_interval

            group = ProcessGroup(self.pid)
            ## Generate the children information before the waitpid call to
            ## avoid a race condition. This way, we know that the pid
            ## is a descendant.

            pid, status = os.waitpid(self.pid, os.WNOHANG)
            if (pid, status) != (0, 0):
                self._handle_exitstatus(status)
                break

            total_time = group.total_time()
            total_vsize = group.total_vsize()

            if real_time >= last_log_time + self.log_interval:
                print "[real-time %d] total_time: %.2f" % (real_time, total_time)
                print "[real-time %d] total_vsize: %.2f" % (real_time, total_vsize)
                last_log_time = real_time

            try_term = (total_time >= self.time_limit or
                        real_time >= self.wall_clock_time_limit or
                        total_vsize > self.mem_limit)
            try_kill = (total_time >= self.time_limit + self.kill_delay or
                        real_time >= 1.5 * self.wall_clock_time_limit +
                                     self.kill_delay or
                        total_vsize > 1.5 * self.mem_limit)

            if try_term and not term_attempted:
                print "aborting children with SIGTERM..."
                print "children found: %s" % group.pids()
                kill_pgrp(self.pid, signal.SIGTERM)
                term_attempted = True
            elif term_attempted and try_kill:
                print "aborting children with SIGKILL..."
                print "children found: %s" % group.pids()
                kill_pgrp(self.pid, signal.SIGKILL)

        # Even if we got here, there may be orphaned children or something
        # we may have missed due to a race condition. Check for that and kill.

        group = ProcessGroup(self.pid)
        if group:
            # If we have reason to suspect someone still lives, first try to
            # kill them nicely and wait a bit.
            print "aborting orphaned children with SIGTERM..."
            print "children found: %s" % group.pids()
            kill_pgrp(self.pid, signal.SIGTERM)
            time.sleep(1)

        # Either way, kill properly for good measure. Note that it's not clear
        # if checking the ProcessGroup for emptiness is reliable, because
        # reading the process table may not be atomic, so for this last blow,
        # we don't do an emptiness test.
        kill_pgrp(self.pid, signal.SIGKILL)

        return self.returncode


if __name__ == "__main__":
    call = Call(['gnome-calculator'], time_limit=6, wall_clock_time_limit=7)
    print 'PID', call.pid
    call.wait()
    print 'RETCODE', call.returncode
