import os
import yaml

import rospy

def load_yaminfo(yaml_path):
    if not os.path.exists(yaml_path):
        print("ERROR")
        return None, None, None
    with open(yaml_path, 'r') as f:
        key = yaml.load(f)
        sendor_addr = key['SENDOR_ADDR']
        recipient_addr_p = key['RECIPIENT_ADDR_P']
        recipient_addr_o = key['RECIPIENT_ADDR_O']
        password = key['PASSWORD']
        return sendor_addr, recipient_addr_p, recipient_addr_o, password
