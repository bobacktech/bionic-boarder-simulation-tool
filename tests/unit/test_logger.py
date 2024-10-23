from augmented_skateboarding_simulator.logger import Logger, NullLogger
import os


def test_null_logger():
    logger = Logger().logger
    assert isinstance(logger, NullLogger)


def test_log_messages_to_file():
    Logger._instance = None
    Logger.enabled = True
    logger = Logger().logger
    log_files = [f for f in os.listdir(".") if f.startswith("skateboard_sim_") and f.endswith(".log")]
    assert len(log_files) > 0, "Log file was not created."
    logger.info("This is an info message")
    logger.info("Test message", value=42)
    logger.error("Error message", error="test error")
    logger.error("Error occurred", error_code=500)
    keywords = ["info", "error"]
    results = {}
    for file in log_files:
        results[file] = [word for word in keywords if word in open(file).read().lower()]
    assert "info" in results[log_files[0]]
    assert "error" in results[log_files[0]]
    for log_file in log_files:
        os.remove(log_file)
