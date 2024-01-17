import os

config_root = os.path.join(os.path.dirname(__file__), "../config")
ROOT_PATH = os.path.dirname(__file__)

from deoxys.utils.log_utils import get_deoxys_logger

get_deoxys_logger()

__version__ = "0.1.0"
