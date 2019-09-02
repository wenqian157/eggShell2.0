'''
Created on 22.11.2016

@author: rustr, jennyd
'''
from __future__ import print_function
import time
import sys
import os

# set the paths to find library
file_dir = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(file_dir, "..", ".."))
sys.path.append(file_dir)
sys.path.append(parent_dir)

import ur_online_control.communication.container as container
from ur_online_control.communication.server import Server
from ur_online_control.communication.client_wrapper import ClientWrapper
from ur_online_control.communication.formatting import format_commands

if len(sys.argv) > 1:
    server_address = sys.argv[1]
    server_port = int(sys.argv[2])
    ur_ip = sys.argv[3]
    print(sys.argv)
else:
    #server_address = "192.168.10.12"
    server_address = "127.0.0.1"
    server_port = 30003
    #ur_ip = "192.168.10.11"
    ur_ip = "127.0.0.1"

def main():

    # start the server
    server = Server(server_address, server_port)
    server.start()
    server.client_ips.update({"UR": ur_ip})

    # create client wrappers, that wrap the underlying communication to the sockets
    gh = ClientWrapper("GH")
    ur = ClientWrapper("UR")

    # wait for the clients to be connected
    gh.wait_for_connected()
    ur.wait_for_connected()

    # now enter fabrication loop
    while True: # and ur and gh connected

        # let gh control if we should continue
        continue_fabrication = gh.wait_for_int() #1 client.send(MSG_INT, int(continue_fabrication))
        print("1: continue_fabrication: %i" % continue_fabrication)
        print ("start fabrication")

        tool_string = gh.wait_for_float_list() #2 client.send(MSG_FLOAT_LIST,tool_string_axis)
        print("2: set tool TCP")
        ur.send_command_tcp(tool_string)

        ### Safe Pose ###
        safety_pose_cmd = gh.wait_for_float_list() #3 client.send(MSG_FLOAT_LIST, safe_in_pose_cmd)
        print("3: safe in pose")
        x, y, z, ax, ay, az, speed, acc = safety_pose_cmd
        ur.send_command_movel([x, y, z, ax, ay, az], v=speed, a=acc)
        ur.send_command_wait(1.0)

        #turn on air and off motor
        ur.send_command_digital_out(4, False)
        ur.send_command_digital_out(5, True)

        ur.wait_for_ready()
        ur.send_command_popup(title='hello!', message='Press ok when you are ready!', blocking=True)
        ur.wait_for_ready()

        #turn on motor
        ur.send_command_digital_out(4, True)

        ur.wait_for_ready()
        ur.send_command_popup(title='hello!', message='Press ok when you are ready!', blocking=True)
        ur.wait_for_ready()

        len_command = gh.wait_for_int() #4 client.send(MSG_INT, len_command)
        print ("4: len command list")

        commands_flattened = gh.wait_for_float_list() #5 client.send(MSG_FLOAT_LIST, commands_flattened)
        print ("5: command list")

        # the commands are formatted according to the sent length
        commands = format_commands(commands_flattened, len_command)
        print("We received %i commands." % len(commands))

        # printing path
        count = 0
        for i in range(0, len(commands)):

            printing_cmd = commands[i]
            x, y, z, ax, ay, az, speed, radius, extrude_bool = printing_cmd

            if extrude_bool == 0:
                count += 1
                ur.send_command_digital_out(4, False)
                ur.send_command_movel([x, y, z, ax, ay, az], v=speed, r=radius)
                if count>2:
                    ur.send_command_digital_out(4, True)
                    ur.send_command_wait(0.7)

            else:
                count = 0
                ur.send_command_movel([x, y, z, ax, ay, az], v=speed, r=radius)

        for i, cmd in enumerate(commands[:-2]):
            ur.wait_for_command_executed(i)
            print("Executed command", i+1)
            print(cmd)

        ### Safe Pose ###
        safety_pose_cmd = gh.wait_for_float_list() #6 client.send(MSG_FLOAT_LIST, safe_out_pose_cmd)
        print ("6: safe out pose")
        x, y, z, ax, ay, az, speed, acc = safety_pose_cmd
        ur.send_command_movel([x, y, z, ax, ay, az], v=speed, a=acc)
        ur.send_command_wait(5.0)

        # turn off motor and extruder
        ur.send_command_digital_out(4, False)
        ur.send_command_wait(0.5)


        ur.wait_for_ready()

        #ur.wait_for_ready()

        print("============================================================")

    ur.quit()
    gh.quit()
    server.close()

    print("Please press a key to terminate the program.")
    junk = sys.stdin.readline()
    print("Done.")

if __name__ == "__main__":
    main()
