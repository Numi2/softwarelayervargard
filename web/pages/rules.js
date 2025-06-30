import { useState, useEffect } from 'react';
import {
  Box,
  VStack,
  HStack,
  Heading,
  Text,
  Badge,
  Button,
  Table,
  Thead,
  Tbody,
  Tr,
  Th,
  Td,
  TableContainer,
  useColorModeValue,
  IconButton,
  Tooltip,
  Select,
  Input,
  Textarea,
  Switch,
  FormControl,
  FormLabel,
  Modal,
  ModalOverlay,
  ModalContent,
  ModalHeader,
  ModalFooter,
  ModalBody,
  ModalCloseButton,
  useDisclosure,
  SimpleGrid,
  Stat,
  StatLabel,
  StatNumber,
  StatHelpText,
  useToast,
  Tabs,
  TabList,
  TabPanels,
  Tab,
  TabPanel,
  Code,
  Alert,
  AlertIcon,
  Divider,
  NumberInput,
  NumberInputField,
  NumberInputStepper,
  NumberIncrementStepper,
  NumberDecrementStepper,
} from '@chakra-ui/react';
import {
  FiPlus,
  FiEdit,
  FiTrash2,
  FiPlay,
  FiPause,
  FiSettings,
  FiCode,
  FiEye,
  FiSave,
  FiRefreshCw,
  FiCheckCircle,
  FiXCircle,
  FiAlertTriangle,
} from 'react-icons/fi';
import { format } from 'date-fns';
import Layout from '../components/Layout';

// Mock rules data - replace with real API calls
const mockRules = [
  {
    id: 'person_detection',
    name: 'Person Detection Alert',
    enabled: true,
    plugin: 'yolov8',
    conditions: {
      class: 'person',
      confidence_gt: 0.7,
      area_gt: 1000,
    },
    actions: {
      type: 'webhook',
      url: 'http://localhost:8000/alert',
      message: 'Person detected with high confidence',
    },
    cooldown: 30,
    max_triggers: 10,
    triggered_count: 42,
    last_triggered: new Date(Date.now() - 300000),
    created_at: new Date(Date.now() - 86400000),
    error_count: 0,
  },
  {
    id: 'vehicle_detection',
    name: 'Vehicle Detection',
    enabled: true,
    plugin: 'yolov8',
    conditions: {
      class: 'vehicle',
      confidence_gt: 0.8,
    },
    actions: {
      type: 'log',
      level: 'info',
      message: 'Vehicle detected',
    },
    cooldown: 60,
    max_triggers: 100,
    triggered_count: 15,
    last_triggered: new Date(Date.now() - 120000),
    created_at: new Date(Date.now() - 172800000),
    error_count: 2,
  },
  {
    id: 'anomaly_detection',
    name: 'Anomaly Detection',
    enabled: false,
    plugin: 'custom_detector',
    conditions: {
      class: 'anomaly',
      confidence_gt: 0.9,
    },
    actions: {
      type: 'email',
      to: 'admin@vargard.ai',
      subject: 'Anomaly Detected',
    },
    cooldown: 300,
    max_triggers: 5,
    triggered_count: 0,
    last_triggered: null,
    created_at: new Date(Date.now() - 259200000),
    error_count: 0,
  },
];

const RuleEditor = ({ rule, isOpen, onClose, onSave }) => {
  const [editedRule, setEditedRule] = useState(rule || {
    name: '',
    enabled: true,
    plugin: 'yolov8',
    conditions: {
      class: '',
      confidence_gt: 0.5,
    },
    actions: {
      type: 'webhook',
      url: '',
    },
    cooldown: 30,
    max_triggers: 10,
  });
  const [yamlMode, setYamlMode] = useState(false);
  const [yamlContent, setYamlContent] = useState('');
  const [validationError, setValidationError] = useState('');

  useEffect(() => {
    if (rule) {
      setEditedRule(rule);
      setYamlContent(JSON.stringify(rule, null, 2));
    }
  }, [rule]);

  const handleSave = () => {
    // Validate rule
    if (!editedRule.name) {
      setValidationError('Rule name is required');
      return;
    }
    if (!editedRule.conditions.class) {
      setValidationError('Detection class is required');
      return;
    }

    setValidationError('');
    onSave(editedRule);
    onClose();
  };

  const handleYamlChange = (value) => {
    setYamlContent(value);
    try {
      const parsed = JSON.parse(value);
      setEditedRule(parsed);
      setValidationError('');
    } catch (error) {
      setValidationError('Invalid JSON format');
    }
  };

  return (
    <Modal isOpen={isOpen} onClose={onClose} size="4xl">
      <ModalOverlay />
      <ModalContent>
        <ModalHeader>{rule ? 'Edit Rule' : 'Create New Rule'}</ModalHeader>
        <ModalCloseButton />
        <ModalBody>
          <Tabs>
            <TabList>
              <Tab>Visual Editor</Tab>
              <Tab>YAML Editor</Tab>
            </TabList>
            
            <TabPanels>
              <TabPanel>
                <VStack spacing={4} align="stretch">
                  {/* Basic Info */}
                  <SimpleGrid columns={2} spacing={4}>
                    <FormControl>
                      <FormLabel>Rule Name</FormLabel>
                      <Input
                        value={editedRule.name}
                        onChange={(e) => setEditedRule(prev => ({ ...prev, name: e.target.value }))}
                        placeholder="Enter rule name"
                      />
                    </FormControl>
                    
                    <FormControl>
                      <FormLabel>Plugin</FormLabel>
                      <Select
                        value={editedRule.plugin}
                        onChange={(e) => setEditedRule(prev => ({ ...prev, plugin: e.target.value }))}
                      >
                        <option value="yolov8">YOLOv8</option>
                        <option value="yolov5">YOLOv5</option>
                        <option value="custom_detector">Custom Detector</option>
                      </Select>
                    </FormControl>
                  </SimpleGrid>

                  <FormControl display="flex" alignItems="center">
                    <FormLabel mb="0">Enabled</FormLabel>
                    <Switch
                      isChecked={editedRule.enabled}
                      onChange={(e) => setEditedRule(prev => ({ ...prev, enabled: e.target.checked }))}
                    />
                  </FormControl>

                  {/* Conditions */}
                  <Box>
                    <Heading size="sm" mb={3}>Detection Conditions</Heading>
                    <SimpleGrid columns={3} spacing={4}>
                      <FormControl>
                        <FormLabel>Detection Class</FormLabel>
                        <Select
                          value={editedRule.conditions?.class || ''}
                          onChange={(e) => setEditedRule(prev => ({
                            ...prev,
                            conditions: { ...prev.conditions, class: e.target.value }
                          }))}
                        >
                          <option value="">Select class</option>
                          <option value="person">Person</option>
                          <option value="vehicle">Vehicle</option>
                          <option value="animal">Animal</option>
                          <option value="package">Package</option>
                          <option value="anomaly">Anomaly</option>
                        </Select>
                      </FormControl>
                      
                      <FormControl>
                        <FormLabel>Min Confidence</FormLabel>
                        <NumberInput
                          value={editedRule.conditions?.confidence_gt || 0.5}
                          onChange={(_, val) => setEditedRule(prev => ({
                            ...prev,
                            conditions: { ...prev.conditions, confidence_gt: val }
                          }))}
                          min={0}
                          max={1}
                          step={0.1}
                        >
                          <NumberInputField />
                          <NumberInputStepper>
                            <NumberIncrementStepper />
                            <NumberDecrementStepper />
                          </NumberInputStepper>
                        </NumberInput>
                      </FormControl>
                      
                      <FormControl>
                        <FormLabel>Min Area (pixels)</FormLabel>
                        <NumberInput
                          value={editedRule.conditions?.area_gt || 0}
                          onChange={(_, val) => setEditedRule(prev => ({
                            ...prev,
                            conditions: { ...prev.conditions, area_gt: val }
                          }))}
                          min={0}
                        >
                          <NumberInputField />
                          <NumberInputStepper>
                            <NumberIncrementStepper />
                            <NumberDecrementStepper />
                          </NumberInputStepper>
                        </NumberInput>
                      </FormControl>
                    </SimpleGrid>
                  </Box>

                  {/* Actions */}
                  <Box>
                    <Heading size="sm" mb={3}>Actions</Heading>
                    <SimpleGrid columns={2} spacing={4}>
                      <FormControl>
                        <FormLabel>Action Type</FormLabel>
                        <Select
                          value={editedRule.actions?.type || 'webhook'}
                          onChange={(e) => setEditedRule(prev => ({
                            ...prev,
                            actions: { ...prev.actions, type: e.target.value }
                          }))}
                        >
                          <option value="webhook">Webhook</option>
                          <option value="email">Email</option>
                          <option value="log">Log</option>
                          <option value="mqtt">MQTT</option>
                        </Select>
                      </FormControl>
                      
                      {editedRule.actions?.type === 'webhook' && (
                        <FormControl>
                          <FormLabel>Webhook URL</FormLabel>
                          <Input
                            value={editedRule.actions?.url || ''}
                            onChange={(e) => setEditedRule(prev => ({
                              ...prev,
                              actions: { ...prev.actions, url: e.target.value }
                            }))}
                            placeholder="http://localhost:8000/webhook"
                          />
                        </FormControl>
                      )}
                      
                      {editedRule.actions?.type === 'email' && (
                        <>
                          <FormControl>
                            <FormLabel>Email To</FormLabel>
                            <Input
                              value={editedRule.actions?.to || ''}
                              onChange={(e) => setEditedRule(prev => ({
                                ...prev,
                                actions: { ...prev.actions, to: e.target.value }
                              }))}
                              placeholder="admin@example.com"
                            />
                          </FormControl>
                          <FormControl>
                            <FormLabel>Subject</FormLabel>
                            <Input
                              value={editedRule.actions?.subject || ''}
                              onChange={(e) => setEditedRule(prev => ({
                                ...prev,
                                actions: { ...prev.actions, subject: e.target.value }
                              }))}
                              placeholder="Alert from Vargard"
                            />
                          </FormControl>
                        </>
                      )}
                    </SimpleGrid>
                  </Box>

                  {/* Settings */}
                  <Box>
                    <Heading size="sm" mb={3}>Rule Settings</Heading>
                    <SimpleGrid columns={2} spacing={4}>
                      <FormControl>
                        <FormLabel>Cooldown (seconds)</FormLabel>
                        <NumberInput
                          value={editedRule.cooldown || 30}
                          onChange={(_, val) => setEditedRule(prev => ({ ...prev, cooldown: val }))}
                          min={0}
                        >
                          <NumberInputField />
                          <NumberInputStepper>
                            <NumberIncrementStepper />
                            <NumberDecrementStepper />
                          </NumberInputStepper>
                        </NumberInput>
                      </FormControl>
                      
                      <FormControl>
                        <FormLabel>Max Triggers per Hour</FormLabel>
                        <NumberInput
                          value={editedRule.max_triggers || 10}
                          onChange={(_, val) => setEditedRule(prev => ({ ...prev, max_triggers: val }))}
                          min={1}
                        >
                          <NumberInputField />
                          <NumberInputStepper>
                            <NumberIncrementStepper />
                            <NumberDecrementStepper />
                          </NumberInputStepper>
                        </NumberInput>
                      </FormControl>
                    </SimpleGrid>
                  </Box>
                </VStack>
              </TabPanel>
              
              <TabPanel>
                <VStack spacing={4} align="stretch">
                  <Text fontSize="sm" color="gray.600">
                    Edit the rule configuration in JSON format. Changes will be reflected in the visual editor.
                  </Text>
                  <Textarea
                    value={yamlContent}
                    onChange={(e) => handleYamlChange(e.target.value)}
                    placeholder="Rule configuration in JSON format"
                    minH="400px"
                    fontFamily="mono"
                    fontSize="sm"
                  />
                </VStack>
              </TabPanel>
            </TabPanels>
          </Tabs>

          {validationError && (
            <Alert status="error" mt={4}>
              <AlertIcon />
              {validationError}
            </Alert>
          )}
        </ModalBody>
        
        <ModalFooter>
          <Button variant="ghost" mr={3} onClick={onClose}>
            Cancel
          </Button>
          <Button colorScheme="blue" onClick={handleSave}>
            Save Rule
          </Button>
        </ModalFooter>
      </ModalContent>
    </Modal>
  );
};

export default function Rules() {
  const [rules, setRules] = useState(mockRules);
  const [selectedRule, setSelectedRule] = useState(null);
  const [isEditorOpen, setIsEditorOpen] = useState(false);
  
  const toast = useToast();
  const cardBg = useColorModeValue('white', 'gray.800');
  const borderColor = useColorModeValue('gray.200', 'gray.600');

  const handleCreateRule = () => {
    setSelectedRule(null);
    setIsEditorOpen(true);
  };

  const handleEditRule = (rule) => {
    setSelectedRule(rule);
    setIsEditorOpen(true);
  };

  const handleSaveRule = (ruleData) => {
    if (selectedRule) {
      // Update existing rule
      setRules(prevRules => 
        prevRules.map(rule => 
          rule.id === selectedRule.id 
            ? { ...ruleData, id: selectedRule.id, created_at: selectedRule.created_at }
            : rule
        )
      );
      toast({
        title: 'Rule Updated',
        description: `Rule "${ruleData.name}" has been updated successfully.`,
        status: 'success',
        duration: 3000,
      });
    } else {
      // Create new rule
      const newRule = {
        ...ruleData,
        id: `rule_${Date.now()}`,
        triggered_count: 0,
        last_triggered: null,
        created_at: new Date(),
        error_count: 0,
      };
      setRules(prevRules => [newRule, ...prevRules]);
      toast({
        title: 'Rule Created',
        description: `Rule "${ruleData.name}" has been created successfully.`,
        status: 'success',
        duration: 3000,
      });
    }
  };

  const handleToggleRule = (ruleId) => {
    setRules(prevRules => 
      prevRules.map(rule => 
        rule.id === ruleId 
          ? { ...rule, enabled: !rule.enabled }
          : rule
      )
    );
    
    const rule = rules.find(r => r.id === ruleId);
    toast({
      title: rule?.enabled ? 'Rule Disabled' : 'Rule Enabled',
      description: `Rule "${rule?.name}" has been ${rule?.enabled ? 'disabled' : 'enabled'}.`,
      status: 'info',
      duration: 3000,
    });
  };

  const handleDeleteRule = (ruleId) => {
    const rule = rules.find(r => r.id === ruleId);
    setRules(prevRules => prevRules.filter(rule => rule.id !== ruleId));
    
    toast({
      title: 'Rule Deleted',
      description: `Rule "${rule?.name}" has been deleted.`,
      status: 'info',
      duration: 3000,
    });
  };

  // Statistics
  const stats = {
    total: rules.length,
    enabled: rules.filter(r => r.enabled).length,
    triggered: rules.filter(r => r.triggered_count > 0).length,
    errors: rules.reduce((sum, r) => sum + r.error_count, 0),
  };

  return (
    <Layout>
      <VStack spacing={6} align="stretch">
        {/* Header */}
        <HStack justify="space-between" align="center">
          <Box>
            <Heading size="lg" mb={2}>Rule Management</Heading>
            <Text color="gray.600">Create and manage AI detection rules</Text>
          </Box>

          <Button
            leftIcon={<FiPlus />}
            colorScheme="blue"
            onClick={handleCreateRule}
          >
            Create Rule
          </Button>
        </HStack>

        {/* Statistics */}
        <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={6}>
          <Stat
            px={4}
            py={5}
            shadow="xl"
            border="1px solid"
            borderColor={borderColor}
            rounded="lg"
            bg={cardBg}
          >
            <StatLabel>Total Rules</StatLabel>
            <StatNumber>{stats.total}</StatNumber>
            <StatHelpText>Configured rules</StatHelpText>
          </Stat>
          
          <Stat
            px={4}
            py={5}
            shadow="xl"
            border="1px solid"
            borderColor={borderColor}
            rounded="lg"
            bg={cardBg}
          >
            <StatLabel>Active Rules</StatLabel>
            <StatNumber>{stats.enabled}</StatNumber>
            <StatHelpText>Currently enabled</StatHelpText>
          </Stat>
          
          <Stat
            px={4}
            py={5}
            shadow="xl"
            border="1px solid"
            borderColor={borderColor}
            rounded="lg"
            bg={cardBg}
          >
            <StatLabel>Triggered Rules</StatLabel>
            <StatNumber>{stats.triggered}</StatNumber>
            <StatHelpText>Have been triggered</StatHelpText>
          </Stat>
          
          <Stat
            px={4}
            py={5}
            shadow="xl"
            border="1px solid"
            borderColor={borderColor}
            rounded="lg"
            bg={cardBg}
          >
            <StatLabel>Total Errors</StatLabel>
            <StatNumber>{stats.errors}</StatNumber>
            <StatHelpText>Across all rules</StatHelpText>
          </Stat>
        </SimpleGrid>

        {/* Rules Table */}
        <Box bg={cardBg} rounded="lg" border="1px" borderColor={borderColor} overflow="hidden">
          <TableContainer>
            <Table variant="simple">
              <Thead bg={useColorModeValue('gray.50', 'gray.700')}>
                <Tr>
                  <Th>Rule Name</Th>
                  <Th>Plugin</Th>
                  <Th>Status</Th>
                  <Th>Triggers</Th>
                  <Th>Last Triggered</Th>
                  <Th>Errors</Th>
                  <Th>Actions</Th>
                </Tr>
              </Thead>
              <Tbody>
                {rules.map((rule) => (
                  <Tr key={rule.id}>
                    <Td>
                      <VStack align="start" spacing={0}>
                        <Text fontWeight="medium">{rule.name}</Text>
                        <Text fontSize="xs" color="gray.500">{rule.id}</Text>
                      </VStack>
                    </Td>
                    <Td>
                      <Badge colorScheme="purple" variant="subtle">
                        {rule.plugin}
                      </Badge>
                    </Td>
                    <Td>
                      <Badge 
                        colorScheme={rule.enabled ? 'green' : 'gray'} 
                        variant={rule.enabled ? 'solid' : 'outline'}
                      >
                        {rule.enabled ? 'Enabled' : 'Disabled'}
                      </Badge>
                    </Td>
                    <Td>
                      <VStack align="start" spacing={0}>
                        <Text fontWeight="medium">{rule.triggered_count}</Text>
                        <Text fontSize="xs" color="gray.500">
                          Max: {rule.max_triggers}/hr
                        </Text>
                      </VStack>
                    </Td>
                    <Td>
                      {rule.last_triggered ? (
                        <VStack align="start" spacing={0}>
                          <Text fontSize="sm">
                            {format(rule.last_triggered, 'MMM dd, HH:mm')}
                          </Text>
                          <Text fontSize="xs" color="gray.500">
                            {Math.floor((Date.now() - rule.last_triggered.getTime()) / 60000)}m ago
                          </Text>
                        </VStack>
                      ) : (
                        <Text color="gray.500" fontSize="sm">Never</Text>
                      )}
                    </Td>
                    <Td>
                      <Badge 
                        colorScheme={rule.error_count > 0 ? 'red' : 'green'} 
                        variant="outline"
                      >
                        {rule.error_count}
                      </Badge>
                    </Td>
                    <Td>
                      <HStack spacing={1}>
                        <Tooltip label="Edit Rule">
                          <IconButton
                            size="sm"
                            icon={<FiEdit />}
                            onClick={() => handleEditRule(rule)}
                            variant="ghost"
                          />
                        </Tooltip>
                        
                        <Tooltip label={rule.enabled ? 'Disable' : 'Enable'}>
                          <IconButton
                            size="sm"
                            icon={rule.enabled ? <FiPause /> : <FiPlay />}
                            onClick={() => handleToggleRule(rule.id)}
                            colorScheme={rule.enabled ? 'orange' : 'green'}
                            variant="ghost"
                          />
                        </Tooltip>
                        
                        <Tooltip label="Delete Rule">
                          <IconButton
                            size="sm"
                            icon={<FiTrash2 />}
                            onClick={() => handleDeleteRule(rule.id)}
                            colorScheme="red"
                            variant="ghost"
                          />
                        </Tooltip>
                      </HStack>
                    </Td>
                  </Tr>
                ))}
              </Tbody>
            </Table>
          </TableContainer>
          
          {rules.length === 0 && (
            <Box p={8} textAlign="center">
              <Text color="gray.500" mb={4}>No rules configured yet</Text>
              <Button leftIcon={<FiPlus />} colorScheme="blue" onClick={handleCreateRule}>
                Create Your First Rule
              </Button>
            </Box>
          )}
        </Box>

        {/* Rule Editor Modal */}
        <RuleEditor
          rule={selectedRule}
          isOpen={isEditorOpen}
          onClose={() => setIsEditorOpen(false)}
          onSave={handleSaveRule}
        />
      </VStack>
    </Layout>
  );
}