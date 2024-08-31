import json
from jsonschema import validate, ValidationError
import os
import pytest
from augmented_skateboarding_simulator.main import AppInputArguments


def test_load_and_validate_app_input_arguments_json():
    script_dir = os.path.dirname(__file__)
    schema_path = os.path.join(script_dir, "../../augmented_skateboarding_simulator/app_input_arguments.schema.json")

    with open(schema_path, "r") as schema_file:
        schema = json.load(schema_file)

    data_path = os.path.join(script_dir, "../app_input_arguments_example.json")
    with open(data_path, "r") as data_file:
        data = json.load(data_file)

    try:
        validate(instance=data, schema=schema)
    except ValidationError as e:
        pytest.fail(f"Schema validation failed: {e}")

    com_port = data["com_port"]
    app_input_arguments = AppInputArguments(**data)
    assert app_input_arguments.com_port == com_port
