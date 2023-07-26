import rospy

def print_error(string):
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    print(FAIL + string + ENDC)

def print_info(string):
    BOLD = '\033[1m'
    ENDC = '\033[0m'
    print(BOLD + string + ENDC)

def print_ok(string):
    OKGREEN = '\033[92m'
    ENDC = '\033[0m'
    print(OKGREEN + string + ENDC)

def is_number(string):
    try:
        num = float(string)
    except:
        return False
    return True

def rinfo(message):
    rospy.loginfo('[DroneSpawner]: ' + message)

def rwarn(message):
    rospy.logwarn('[DroneSpawner]: ' + message)

def rerr(message):
    rospy.logerr('[DroneSpawner]: ' + message)

