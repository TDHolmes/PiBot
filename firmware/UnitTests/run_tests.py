#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import subprocess
import os
import sys
from threading import Thread
from Queue import Queue, Empty
import time


def run_unit_test(source_code_list, output_name, unity_path, verbose=False):
    BASE_CMD_START = "gcc  -I {0} {0}unity.c ".format(unity_path)
    # build up the command with the source and output name
    abs_path_list = [os.path.abspath(src_file) for src_file in source_code_list]
    cmd = BASE_CMD_START + "{} -o {}".format(" ".join(abs_path_list), output_name)
    if verbose:
        cmd += " -D VERBOSE_OUTPUT"
    stdout, stderr, retval = run_command(cmd)

    # run the output program
    cmd = "./{}".format(output_name)
    stdout, stderr, retval = run_command(cmd)

    run_command("rm {}".format(output_name), print_output=verbose)

    return retval


def main(verbose, unity_path):
    # run all tests!
    retval = run_unit_test(["dummy_test.c"], "test_dummy_test", unity_path, verbose)
    retval += run_unit_test(["../MotorController/motor_calc.c", "test_motor_calc.c"], "test_motor_calc",
                            unity_path, verbose)
    sys.exit(retval)


def run_command(cmd, print_output=True):
    """
    Kicks off a subprocess to run and accumilate the stdout of the process.
    """

    # function we can use to queue up the stdout/err output
    def enqueue_output(out, queue):
        for line in iter(out.readline, b''):
            queue.put(line)
        out.close()

    if print_output:
        print(" -> {}".format(cmd))

    # ge the process and stdout/err queues put together
    proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    q_stdout = Queue()
    q_stderr = Queue()
    t_stdout = Thread(target=enqueue_output, args=(proc.stdout, q_stdout))
    t_stderr = Thread(target=enqueue_output, args=(proc.stderr, q_stderr))
    t_stderr.daemon = True  # thread dies with the program
    t_stdout.daemon = True
    t_stdout.start()
    t_stderr.start()
    stdout = ""
    stderr = ""

    # read stdout and stderr without blocking
    finished = False
    while True:
        retval = proc.poll()
        try:
            line_stdout = ""
            while True:
                line_stdout += q_stdout.get_nowait()
        except Empty:
            pass
        # accumilate stdout and print if we should
        stdout += line_stdout
        if print_output and line_stdout != "":
            print(line_stdout.rstrip("\n"))

        try:
            line_stderr = ""
            while True:
                line_stderr += q_stderr.get_nowait()
        except Empty:
            pass
        # accumilate stderr and print if we should
        stderr += line_stderr
        if print_output and line_stderr != "":
            print(line_stderr.rstrip("\n"))

        if finished:
            return stdout, stderr, retval

        if retval is not None:
            finished = True
            # give stdout and stderr time
            time.sleep(0.25)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("-u", "--unity-path", type=str,
                        default="/usr/local/resources/tools/Unity/src/",
                        help="Path to the Unity C Unit testing framework.")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose output.")

    args = parser.parse_args()

    main(args.verbose, args.unity_path)
