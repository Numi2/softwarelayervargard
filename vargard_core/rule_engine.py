"""
Enhanced rule engine for Vargard event processing with advanced condition evaluation.
"""

import re
import time
from typing import Dict, List, Any, Optional
from enum import Enum

class ConditionOperator(Enum):
    """Supported condition operators."""
    EQUALS = "equals"
    NOT_EQUALS = "not_equals"
    GREATER_THAN = "gt"
    GREATER_THAN_OR_EQUAL = "gte"
    LESS_THAN = "lt"
    LESS_THAN_OR_EQUAL = "lte"
    CONTAINS = "contains"
    NOT_CONTAINS = "not_contains"
    REGEX_MATCH = "regex_match"
    IN_LIST = "in"
    NOT_IN_LIST = "not_in"

class LogicalOperator(Enum):
    """Logical operators for combining conditions."""
    AND = "and"
    OR = "or"
    NOT = "not"

class RuleEngine:
    """Enhanced rule engine with complex condition evaluation."""
    
    def __init__(self):
        self.rules = []
        self.rule_stats = {}
        
    def load_rules(self, rules_config: List[Dict[str, Any]]) -> bool:
        """
        Load and validate rules from configuration.
        
        Args:
            rules_config: List of rule dictionaries
            
        Returns:
            bool: True if all rules loaded successfully
        """
        self.rules = []
        self.rule_stats = {}
        
        for rule_config in rules_config:
            try:
                rule = self._parse_rule(rule_config)
                if self._validate_rule(rule):
                    self.rules.append(rule)
                    self.rule_stats[rule['name']] = {
                        'triggered_count': 0,
                        'last_triggered': None,
                        'error_count': 0
                    }
                else:
                    print(f"Invalid rule: {rule_config.get('name', 'unnamed')}")
                    return False
            except Exception as e:
                print(f"Error parsing rule {rule_config.get('name', 'unnamed')}: {e}")
                return False
                
        return True
    
    def _parse_rule(self, rule_config: Dict[str, Any]) -> Dict[str, Any]:
        """Parse rule configuration into internal format."""
        rule = {
            'name': rule_config.get('name', 'unnamed_rule'),
            'description': rule_config.get('description', ''),
            'plugin': rule_config.get('plugin'),
            'conditions': self._parse_conditions(rule_config.get('condition', {})),
            'actions': rule_config.get('action', {}),
            'enabled': rule_config.get('enabled', True),
            'cooldown': rule_config.get('cooldown', 0),  # seconds
            'max_triggers': rule_config.get('max_triggers', None),
            'priority': rule_config.get('priority', 'medium')
        }
        return rule
    
    def _parse_conditions(self, condition_config: Dict[str, Any]) -> Dict[str, Any]:
        """Parse condition configuration with support for complex logic."""
        if not condition_config:
            return {}
            
        # Handle logical operators (and, or, not)
        if 'and' in condition_config:
            return {
                'operator': LogicalOperator.AND,
                'conditions': [self._parse_conditions(c) for c in condition_config['and']]
            }
        elif 'or' in condition_config:
            return {
                'operator': LogicalOperator.OR,
                'conditions': [self._parse_conditions(c) for c in condition_config['or']]
            }
        elif 'not' in condition_config:
            return {
                'operator': LogicalOperator.NOT,
                'condition': self._parse_conditions(condition_config['not'])
            }
        else:
            # Simple condition
            return condition_config
    
    def _validate_rule(self, rule: Dict[str, Any]) -> bool:
        """Validate rule structure and content."""
        # Check required fields
        if not rule.get('name'):
            return False
            
        # Validate actions
        action = rule.get('actions', {})
        if not action.get('type'):
            return False
            
        # Validate cooldown
        cooldown = rule.get('cooldown', 0)
        if not isinstance(cooldown, (int, float)) or cooldown < 0:
            return False
            
        return True
    
    def evaluate_event(self, event_data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Evaluate event against all rules and return triggered actions.
        
        Args:
            event_data: Event data from inference
            
        Returns:
            List of triggered rule actions
        """
        triggered_actions = []
        current_time = time.time()
        
        for rule in self.rules:
            if not rule.get('enabled', True):
                continue
                
            rule_name = rule['name']
            
            try:
                # Check plugin filter
                if rule.get('plugin') and event_data.get('plugin') != rule['plugin']:
                    continue
                
                # Check cooldown
                stats = self.rule_stats.get(rule_name, {})
                last_triggered = stats.get('last_triggered')
                cooldown = rule.get('cooldown', 0)
                
                if last_triggered and (current_time - last_triggered) < cooldown:
                    continue
                
                # Check max triggers
                max_triggers = rule.get('max_triggers')
                if max_triggers and stats.get('triggered_count', 0) >= max_triggers:
                    continue
                
                # Evaluate conditions
                if self._evaluate_conditions(rule.get('conditions', {}), event_data):
                    # Rule triggered!
                    triggered_actions.append({
                        'rule': rule,
                        'event_data': event_data,
                        'timestamp': current_time
                    })
                    
                    # Update statistics
                    self.rule_stats[rule_name]['triggered_count'] += 1
                    self.rule_stats[rule_name]['last_triggered'] = current_time
                    
            except Exception as e:
                print(f"Error evaluating rule {rule_name}: {e}")
                self.rule_stats[rule_name]['error_count'] += 1
                
        return triggered_actions
    
    def _evaluate_conditions(self, conditions: Dict[str, Any], event_data: Dict[str, Any]) -> bool:
        """Evaluate conditions against event data."""
        if not conditions:
            return True
            
        # Handle logical operators
        operator = conditions.get('operator')
        if operator == LogicalOperator.AND:
            return all(self._evaluate_conditions(c, event_data) for c in conditions['conditions'])
        elif operator == LogicalOperator.OR:
            return any(self._evaluate_conditions(c, event_data) for c in conditions['conditions'])
        elif operator == LogicalOperator.NOT:
            return not self._evaluate_conditions(conditions['condition'], event_data)
        
        # Evaluate simple conditions
        return self._evaluate_simple_condition(conditions, event_data)
    
    def _evaluate_simple_condition(self, condition: Dict[str, Any], event_data: Dict[str, Any]) -> bool:
        """Evaluate a simple condition."""
        # Check detection-based conditions
        detections = event_data.get('detections', [])
        
        for detection in detections:
            if self._check_detection_condition(condition, detection):
                return True
                
        # Check event-level conditions
        return self._check_event_condition(condition, event_data)
    
    def _check_detection_condition(self, condition: Dict[str, Any], detection: Dict[str, Any]) -> bool:
        """Check condition against a single detection."""
        # Class condition
        if 'class' in condition:
            if not self._compare_values(detection.get('class'), condition['class'], ConditionOperator.EQUALS):
                return False
        
        # Confidence threshold
        if 'confidence_gt' in condition:
            if not self._compare_values(detection.get('confidence', 0), condition['confidence_gt'], ConditionOperator.GREATER_THAN):
                return False
                
        if 'confidence_lt' in condition:
            if not self._compare_values(detection.get('confidence', 0), condition['confidence_lt'], ConditionOperator.LESS_THAN):
                return False
        
        # Bounding box conditions
        bbox = detection.get('bbox', [])
        if len(bbox) >= 4:
            x, y, w, h = bbox[:4]
            
            # Area condition
            if 'min_area' in condition:
                area = w * h
                if not self._compare_values(area, condition['min_area'], ConditionOperator.GREATER_THAN_OR_EQUAL):
                    return False
                    
            # Position conditions
            if 'x_range' in condition:
                x_min, x_max = condition['x_range']
                if not (x_min <= x <= x_max):
                    return False
                    
            if 'y_range' in condition:
                y_min, y_max = condition['y_range']
                if not (y_min <= y <= y_max):
                    return False
        
        return True
    
    def _check_event_condition(self, condition: Dict[str, Any], event_data: Dict[str, Any]) -> bool:
        """Check condition against event-level data."""
        # Detection count condition
        if 'detection_count_gt' in condition:
            count = len(event_data.get('detections', []))
            if not self._compare_values(count, condition['detection_count_gt'], ConditionOperator.GREATER_THAN):
                return False
        
        # Sensor condition
        if 'sensor_id' in condition:
            if not self._compare_values(event_data.get('sensor_id'), condition['sensor_id'], ConditionOperator.EQUALS):
                return False
        
        # Time-based conditions
        if 'time_range' in condition:
            current_hour = time.localtime().tm_hour
            start_hour, end_hour = condition['time_range']
            if not (start_hour <= current_hour <= end_hour):
                return False
        
        return True
    
    def _compare_values(self, actual: Any, expected: Any, operator: ConditionOperator) -> bool:
        """Compare two values using the specified operator."""
        try:
            if operator == ConditionOperator.EQUALS:
                return actual == expected
            elif operator == ConditionOperator.NOT_EQUALS:
                return actual != expected
            elif operator == ConditionOperator.GREATER_THAN:
                return actual > expected
            elif operator == ConditionOperator.GREATER_THAN_OR_EQUAL:
                return actual >= expected
            elif operator == ConditionOperator.LESS_THAN:
                return actual < expected
            elif operator == ConditionOperator.LESS_THAN_OR_EQUAL:
                return actual <= expected
            elif operator == ConditionOperator.CONTAINS:
                return str(expected).lower() in str(actual).lower()
            elif operator == ConditionOperator.NOT_CONTAINS:
                return str(expected).lower() not in str(actual).lower()
            elif operator == ConditionOperator.REGEX_MATCH:
                return bool(re.search(str(expected), str(actual)))
            elif operator == ConditionOperator.IN_LIST:
                return actual in expected
            elif operator == ConditionOperator.NOT_IN_LIST:
                return actual not in expected
        except Exception:
            return False
            
        return False
    
    def get_rule_stats(self) -> Dict[str, Any]:
        """Get statistics for all rules."""
        return self.rule_stats.copy()
    
    def reset_rule_stats(self, rule_name: Optional[str] = None):
        """Reset statistics for a specific rule or all rules."""
        if rule_name:
            if rule_name in self.rule_stats:
                self.rule_stats[rule_name] = {
                    'triggered_count': 0,
                    'last_triggered': None,
                    'error_count': 0
                }
        else:
            for rule_name in self.rule_stats:
                self.rule_stats[rule_name] = {
                    'triggered_count': 0,
                    'last_triggered': None,
                    'error_count': 0
                }