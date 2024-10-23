import logging
import structlog
from datetime import datetime


class NullLogger:
    """A logger that does nothing."""

    def __getattr__(self, name):
        def method(*args, **kwargs):
            pass

        return method


class Logger:
    enabled = False
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(Logger, cls).__new__(cls)
            if Logger.enabled:
                cls._instance._configure()
            else:
                cls._instance._logger = NullLogger()
        return cls._instance

    def _configure(self):
        # Create timestamp for the log file name
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_file_name = f"skateboard_sim_{timestamp}.log"

        # Configure standard logging first
        # Ensure configuration is applied with force=True
        logging.basicConfig(level=logging.INFO, format="%(message)s", force=True)

        # Create and configure file handler
        file_handler = logging.FileHandler(log_file_name)
        console_handler = logging.StreamHandler()

        # Create formatter and add it to handlers
        formatter = logging.Formatter("%(message)s")
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)

        # Get root logger and add handlers
        root_logger = logging.getLogger()
        root_logger.handlers = []  # Clear existing handlers
        root_logger.addHandler(file_handler)
        root_logger.addHandler(console_handler)

        # Configure structlog
        structlog.configure(
            processors=[
                structlog.stdlib.add_log_level,
                structlog.stdlib.PositionalArgumentsFormatter(),
                structlog.processors.TimeStamper(fmt="iso"),
                structlog.processors.JSONRenderer(indent=None),
            ],
            context_class=dict,
            logger_factory=structlog.stdlib.LoggerFactory(),
            wrapper_class=structlog.stdlib.BoundLogger,
            cache_logger_on_first_use=True,
        )

        self._logger = structlog.get_logger()

    @property
    def logger(self):
        return self._logger
