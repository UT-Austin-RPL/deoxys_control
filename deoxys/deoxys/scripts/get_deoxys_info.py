
import pprint
from termcolor import colored

from deoxys.utils.config_utils import get_available_controller_configs


def default_controller_list():
    config_dict = get_available_controller_configs()   
    print(colored('Available controllers:', "green", attrs=['reverse']))

    for k in config_dict.keys():
        print(f"  - {k}")

    
def default_all_controller_info():
    config_dict = get_available_controller_configs()
    pp = pprint.PrettyPrinter(indent=2)
    print(colored('Available controllers:', "green", attrs=['reverse']))
    for k in config_dict.keys():
        print(colored(f"{k}:", "green"))
        pp.pprint(config_dict[k])