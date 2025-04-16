import os
import yaml

def test_rules_file_exists():
    # Ensure the example rules.yaml is present and parses correctly
    path = os.path.abspath('rules.yaml')
    assert os.path.exists(path), f'rules.yaml not found at {path}'
    cfg = yaml.safe_load(open(path))
    assert 'rules' in cfg and isinstance(cfg['rules'], list), 'rules key missing or invalid'
    # Check at least one rule has required fields
    first = cfg['rules'][0]
    assert 'name' in first and 'plugin' in first and 'condition' in first and 'action' in first