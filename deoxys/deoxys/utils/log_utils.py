import logging
import logging.config
import os

from termcolor import colored

from deoxys.utils.config_utils import config_root
from deoxys.utils.yaml_config import YamlConfig


class DeoxysDefaultLogger:
    def __init__(self, logger_config_path):
        if logger_config_path is None:
            logger_config_path = os.path.join(config_root, "deoxys_default_logger.yml")
        config = YamlConfig(logger_config_path).as_easydict()
        os.makedirs("logs", exist_ok=True)
        logging.config.dictConfig(config)


class ProjectDefaultLogger:
    def __init__(self, logger_config_path, project_name):
        if logger_config_path is None:
            logger_config_path = os.path.join(config_root, "deoxys_default_logger.yml")
        config = YamlConfig(logger_config_path).as_easydict()
        config["loggers"][project_name] = config["loggers"]["project"]
        os.makedirs("logs", exist_ok=True)
        logging.config.dictConfig(config)


class DeoxysColorFormatter(logging.Formatter):
    format_str = "[Deoxys %(levelname)s] "
    message_str = "%(message)s (%(filename)s:%(lineno)d)"
    FORMATS = {
        logging.DEBUG: format_str + message_str,
        logging.INFO: colored(format_str, "grey", attrs=["bold"]) + message_str,
        logging.WARNING: colored(format_str, "yellow", attrs=["bold"]) + message_str,
        logging.ERROR: colored(format_str, "red", attrs=["bold"]) + message_str,
        logging.CRITICAL: colored(format_str, "red", attrs=["bold", "reverse"])
        + message_str,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


class DeoxysExampleColorFormatter(logging.Formatter):
    format_str = "[Deoxys Examples %(levelname)s] "
    message_str = "%(message)s (Ln %(lineno)d)"
    FORMATS = {
        logging.DEBUG: format_str + message_str,
        logging.INFO: colored(format_str, "grey", attrs=["bold"]) + message_str,
        logging.WARNING: colored(format_str, "yellow", attrs=["bold"]) + message_str,
        logging.ERROR: colored(format_str, "red", attrs=["bold"]) + message_str,
        logging.CRITICAL: colored(format_str, "red", attrs=["bold", "reverse"])
        + message_str,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


class ProjectColorFormatter(logging.Formatter):
    """This color format is for logging user's project wise information"""

    format_str = "[Project %(levelname)s] "
    message_str = "%(message)s"
    FORMATS = {
        logging.DEBUG: format_str + message_str,
        logging.INFO: colored(format_str, "green", attrs=["bold"]) + message_str,
        logging.WARNING: colored(format_str, "yellow", attrs=["bold"]) + message_str,
        logging.ERROR: colored(format_str, "red", attrs=["bold"]) + message_str,
        logging.CRITICAL: colored(format_str, "red", attrs=["bold", "reverse"])
        + message_str,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


def get_deoxys_logger(logger_config_path=None):
    """This function returns a logger that follows the deoxys convention"""
    DeoxysDefaultLogger(logger_config_path)
    logger = logging.getLogger("deoxys")
    return logger


def get_deoxys_example_logger(logger_config_path=None):
    """This function returns a logger that follows the deoxys convention"""
    DeoxysDefaultLogger(logger_config_path)
    logger = logging.getLogger("deoxys_examples")
    return logger


def get_project_logger(project_name="project", logger_config_path=None):
    """This function returns a logger that follows the deoxys convention"""
    ProjectDefaultLogger(logger_config_path, project_name)
    logger = logging.getLogger(project_name)
    return logger
