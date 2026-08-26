import os
import sys
import random
import time
import subprocess
import threading
import json


leader_finished = False
follower_finished = False
lock = threading.Lock()


def main():
    # Read follower ip, log filename and object name
    config = json.load(open("leader_config.json", "r"))
    follower_ip = config["remote_ip"] 
    # time_log_filename = config["time_log_filename"] 
    # time_log_file_path = f"/home/panda/vla_finetune/build/{time_log_filename}" 
    task_name = config["task_name"]

    session_time = time.strftime("%Y%m%d_%H%M%S")
    session_id = f"{task_name}_{session_time}"
    print("Session ID:", session_id)

    num_trials = 50
    for trial in range(num_trials):
        # Kill previous process before execution   
        try: 
            kill_follower_process(follower_ip)
            kill_leader_process()
        except:
            print("No previous process to kill.")
            pass
 
        # print info for new trial
        #print(f"Trial {trial+1}: Wiggle {wiggle_mode} is ready to start ...")
        print(f"Trial {trial+1} is ready to start ...")
        input("Press Enter to start the trial.")
        
        
        #execute_trial(follower_ip, wiggle_mode, participant_name, object_name, trial)
        #execute_trial(follower_ip)
        execute_trial(follower_ip, session_id, trial+1)

    ### End Exeriment for One Object ###
        
#def execute_trial(follower_ip,  wiggle_mode, participant_name, object_name, trial):   
def execute_trial(follower_ip, session_id, trial_idx):         
    
    global leader_finished, follower_finished
    
    leader_finished = False 
    follower_finished = False

    try:   
        # Choose trial condition
        
        leader_cmd = f"cd /home/panda/vla_finetune/build && ./TelePandaTDPA2010AsWhole 192.168.3.100 l {session_id} {trial_idx}" 
        follower_cmd = f"ssh panda@{follower_ip} 'cd /home/panda/vla_finetune/build && ./TelePandaTDPA2010AsWhole 192.168.3.100 f {session_id} {trial_idx}'"

        #leader_cmd = f"cd /home/panda/vla_finetune/build && ./TelePandaTDPA2010AsWhole 192.168.3.100 l {wiggle_mode} {object_name}" 
        #follower_cmd = f"ssh panda@{follower_ip} 'cd /home/panda/vla_finetune/build && ./TelePandaTDPA2010AsWhole 192.168.3.100 f {wiggle_mode} {object_name}'"
        
        # leader thread
        leader_thread = threading.Thread(target=leader_run, args=(leader_cmd,))
        leader_thread.start()
        # follower thread
        follower_thread = threading.Thread(target=follower_run, args=(follower_cmd, follower_ip))
        follower_thread.start()


        leader_thread.join()
        follower_thread.join()

       
    except KeyboardInterrupt:
        kill_follower_process(follower_ip)
        kill_leader_process()
        print("All processes terminated. Exiting.")
        sys.exit(0)

def leader_run(leader_cmd):
    
    global leader_finished

    print("Starting leader...")
    leader_process = subprocess.Popen(leader_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in leader_process.stdout:
        print(f"Leader output: {line.strip()}") 

    return_code = leader_process.wait()
    print(f"Leader return: {return_code}")
    print("Leader finished.")

    with lock:
        leader_finished = True 

def follower_run(follower_cmd, follower_ip):
    
    global follower_finished

    print("Starting follower...")
    follower_process = subprocess.Popen(follower_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in follower_process.stdout:
        print(f"Follower output: {line.strip()}") 

    return_code = follower_process.wait()
    print(f"Follower return: {return_code}")
    print("Follower finished.")

    with lock:
        follower_finished = True 

        
def kill_leader_process():
    print("Killing leader processes...")
    os.system("pkill -9 -f TelePandaTDPA2010AsWhole") 
    time.sleep(1)

def kill_follower_process(follower_ip):
    print("Killing follower processes on remote machine...")
    os.system(f"ssh panda@{follower_ip} 'pkill -9 -f TelePandaTDPA2010AsWhole'") 
    time.sleep(1)



if __name__ == "__main__":
    main()