import logging.config

logging_config = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "standard": {"format": "%(asctime)s [%(levelname)s] %(name)s: %(message)s"}
    },
    "handlers": {"default": {"level": "INFO", "class": "logging.StreamHandler"}},
    "loggers": {"": {"handlers": ["default"], "level": "ERROR", "propagate": True}},
}

logging.config.dictConfig(logging_config)
