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
            cls._instance._configure()
        return cls._instance

    def _configure(self):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_file_name = f"skateboard_sim_{timestamp}.log"

        logging.basicConfig(
            level=logging.INFO,  # Set the logging level
            format="%(message)s",  # Define the log message format
            handlers=[
                logging.FileHandler(log_file_name),  # Log to a file named 'app.log'
                logging.StreamHandler(),  # Optionally, also log to the console
            ],
        )

        structlog.configure(
            processors=[
                structlog.processors.TimeStamper(fmt="iso"),  # Add timestamp to logs
                structlog.processors.JSONRenderer(),  # Render logs as JSON
            ],
            context_class=dict,
            logger_factory=structlog.stdlib.LoggerFactory(),
            wrapper_class=structlog.stdlib.BoundLogger,
            cache_logger_on_first_use=True,
        )

        # Create a logger
        if Logger.enabled:
            self._logger = structlog.get_logger()
        else:
            self._logger = NullLogger()

    @property
    def logger(self):
        return self._logger
