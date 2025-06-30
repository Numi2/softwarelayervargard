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
  StatArrow,
  useToast,
  Progress,
  Tabs,
  TabList,
  TabPanels,
  Tab,
  TabPanel,
  Code,
  Alert,
  AlertIcon,
  CircularProgress,
  CircularProgressLabel,
  Textarea,
  NumberInput,
  NumberInputField,
  NumberInputStepper,
  NumberIncrementStepper,
  NumberDecrementStepper,
  Accordion,
  AccordionItem,
  AccordionButton,
  AccordionPanel,
  AccordionIcon,
} from '@chakra-ui/react';
import {
  FiSettings,
  FiPlay,
  FiPause,
  FiRefreshCw,
  FiDownload,
  FiUpload,
  FiTrash2,
  FiEye,
  FiCpu,
  FiActivity,
  FiBarChart3,
  FiCheckCircle,
  FiXCircle,
  FiAlertTriangle,
  FiPackage,
} from 'react-icons/fi';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip as RechartsTooltip, ResponsiveContainer, AreaChart, Area } from 'recharts';
import { format, subMinutes } from 'date-fns';
import Layout from '../components/Layout';

// Mock plugins data - replace with real API calls
const mockPlugins = [
  {
    id: 'yolov8',
    name: 'YOLOv8 Object Detection',
    version: '1.2.0',
    status: 'running',
    enabled: true,
    author: 'Ultralytics',
    description: 'State-of-the-art object detection with YOLOv8 architecture',
    inference_count: 15420,
    average_inference_time: 0.045,
    fps: 22.1,
    accuracy: 0.89,
    gpu_usage: 78,
    memory_usage: 2.1, // GB
    error_count: 3,
    last_error: 'GPU memory allocation failed',
    config: {
      confidence_threshold: 0.5,
      nms_threshold: 0.4,
      max_detections: 100,
      input_size: 640,
      device: 'cuda:0',
    },
    performance_history: Array.from({ length: 60 }, (_, i) => ({
      time: format(subMinutes(new Date(), 59 - i), 'HH:mm'),
      fps: 20 + Math.random() * 8,
      latency: 0.03 + Math.random() * 0.03,
      gpu_usage: 70 + Math.random() * 20,
    })),
  },
  {
    id: 'yolov5',
    name: 'YOLOv5 Object Detection',
    version: '7.0.13',
    status: 'stopped',
    enabled: false,
    author: 'Ultralytics',
    description: 'Proven object detection model with excellent performance',
    inference_count: 8750,
    average_inference_time: 0.038,
    fps: 0,
    accuracy: 0.86,
    gpu_usage: 0,
    memory_usage: 0,
    error_count: 0,
    last_error: null,
    config: {
      confidence_threshold: 0.25,
      nms_threshold: 0.45,
      max_detections: 1000,
      input_size: 640,
      device: 'cuda:0',
    },
    performance_history: [],
  },
  {
    id: 'custom_detector',
    name: 'Custom Anomaly Detector',
    version: '0.3.1',
    status: 'error',
    enabled: true,
    author: 'Vargard Team',
    description: 'Custom-trained anomaly detection for specific use cases',
    inference_count: 234,
    average_inference_time: 0.125,
    fps: 0,
    accuracy: 0.72,
    gpu_usage: 0,
    memory_usage: 0,
    error_count: 15,
    last_error: 'Model file not found: /models/anomaly_v3.pth',
    config: {
      confidence_threshold: 0.8,
      model_path: '/models/anomaly_v3.pth',
      preprocessing: 'normalize',
      batch_size: 1,
    },
    performance_history: [],
  },
];

const PluginConfigModal = ({ plugin, isOpen, onClose, onSave }) => {
  const [config, setConfig] = useState(plugin?.config || {});
  const [validationError, setValidationError] = useState('');

  useEffect(() => {
    if (plugin) {
      setConfig(plugin.config);
    }
  }, [plugin]);

  const handleSave = () => {
    // Basic validation
    if (config.confidence_threshold < 0 || config.confidence_threshold > 1) {
      setValidationError('Confidence threshold must be between 0 and 1');
      return;
    }

    setValidationError('');
    onSave(plugin.id, config);
    onClose();
  };

  if (!plugin) return null;

  return (
    <Modal isOpen={isOpen} onClose={onClose} size="xl">
      <ModalOverlay />
      <ModalContent>
        <ModalHeader>Configure {plugin.name}</ModalHeader>
        <ModalCloseButton />
        <ModalBody>
          <VStack spacing={4} align="stretch">
            <Alert status="info">
              <AlertIcon />
              Changes will take effect after restarting the plugin.
            </Alert>

            <Accordion allowToggle>
              <AccordionItem>
                <AccordionButton>
                  <Box flex="1" textAlign="left">
                    <Text fontWeight="bold">Detection Settings</Text>
                  </Box>
                  <AccordionIcon />
                </AccordionButton>
                <AccordionPanel pb={4}>
                  <SimpleGrid columns={2} spacing={4}>
                    <FormControl>
                      <FormLabel>Confidence Threshold</FormLabel>
                      <NumberInput
                        value={config.confidence_threshold || 0.5}
                        onChange={(_, val) => setConfig(prev => ({ ...prev, confidence_threshold: val }))}
                        min={0}
                        max={1}
                        step={0.05}
                      >
                        <NumberInputField />
                        <NumberInputStepper>
                          <NumberIncrementStepper />
                          <NumberDecrementStepper />
                        </NumberInputStepper>
                      </NumberInput>
                    </FormControl>

                    {config.nms_threshold !== undefined && (
                      <FormControl>
                        <FormLabel>NMS Threshold</FormLabel>
                        <NumberInput
                          value={config.nms_threshold || 0.4}
                          onChange={(_, val) => setConfig(prev => ({ ...prev, nms_threshold: val }))}
                          min={0}
                          max={1}
                          step={0.05}
                        >
                          <NumberInputField />
                          <NumberInputStepper>
                            <NumberIncrementStepper />
                            <NumberDecrementStepper />
                          </NumberInputStepper>
                        </NumberInput>
                      </FormControl>
                    )}

                    {config.max_detections !== undefined && (
                      <FormControl>
                        <FormLabel>Max Detections</FormLabel>
                        <NumberInput
                          value={config.max_detections || 100}
                          onChange={(_, val) => setConfig(prev => ({ ...prev, max_detections: val }))}
                          min={1}
                          max={1000}
                        >
                          <NumberInputField />
                          <NumberInputStepper>
                            <NumberIncrementStepper />
                            <NumberDecrementStepper />
                          </NumberInputStepper>
                        </NumberInput>
                      </FormControl>
                    )}

                    {config.input_size !== undefined && (
                      <FormControl>
                        <FormLabel>Input Size</FormLabel>
                        <Select
                          value={config.input_size || 640}
                          onChange={(e) => setConfig(prev => ({ ...prev, input_size: parseInt(e.target.value) }))}
                        >
                          <option value={320}>320x320</option>
                          <option value={416}>416x416</option>
                          <option value={512}>512x512</option>
                          <option value={640}>640x640</option>
                          <option value={832}>832x832</option>
                        </Select>
                      </FormControl>
                    )}
                  </SimpleGrid>
                </AccordionPanel>
              </AccordionItem>

              <AccordionItem>
                <AccordionButton>
                  <Box flex="1" textAlign="left">
                    <Text fontWeight="bold">Hardware Settings</Text>
                  </Box>
                  <AccordionIcon />
                </AccordionButton>
                <AccordionPanel pb={4}>
                  <SimpleGrid columns={2} spacing={4}>
                    {config.device !== undefined && (
                      <FormControl>
                        <FormLabel>Device</FormLabel>
                        <Select
                          value={config.device || 'cuda:0'}
                          onChange={(e) => setConfig(prev => ({ ...prev, device: e.target.value }))}
                        >
                          <option value="cpu">CPU</option>
                          <option value="cuda:0">GPU (CUDA:0)</option>
                          <option value="cuda:1">GPU (CUDA:1)</option>
                        </Select>
                      </FormControl>
                    )}

                    {config.batch_size !== undefined && (
                      <FormControl>
                        <FormLabel>Batch Size</FormLabel>
                        <NumberInput
                          value={config.batch_size || 1}
                          onChange={(_, val) => setConfig(prev => ({ ...prev, batch_size: val }))}
                          min={1}
                          max={32}
                        >
                          <NumberInputField />
                          <NumberInputStepper>
                            <NumberIncrementStepper />
                            <NumberDecrementStepper />
                          </NumberInputStepper>
                        </NumberInput>
                      </FormControl>
                    )}
                  </SimpleGrid>
                </AccordionPanel>
              </AccordionItem>

              {config.model_path && (
                <AccordionItem>
                  <AccordionButton>
                    <Box flex="1" textAlign="left">
                      <Text fontWeight="bold">Model Settings</Text>
                    </Box>
                    <AccordionIcon />
                  </AccordionButton>
                  <AccordionPanel pb={4}>
                    <FormControl>
                      <FormLabel>Model Path</FormLabel>
                      <Input
                        value={config.model_path || ''}
                        onChange={(e) => setConfig(prev => ({ ...prev, model_path: e.target.value }))}
                        placeholder="/path/to/model.pth"
                      />
                    </FormControl>
                  </AccordionPanel>
                </AccordionItem>
              )}
            </Accordion>

            {validationError && (
              <Alert status="error">
                <AlertIcon />
                {validationError}
              </Alert>
            )}
          </VStack>
        </ModalBody>
        
        <ModalFooter>
          <Button variant="ghost" mr={3} onClick={onClose}>
            Cancel
          </Button>
          <Button colorScheme="blue" onClick={handleSave}>
            Save Configuration
          </Button>
        </ModalFooter>
      </ModalContent>
    </Modal>
  );
};

const PluginDetailModal = ({ plugin, isOpen, onClose }) => {
  if (!plugin) return null;

  return (
    <Modal isOpen={isOpen} onClose={onClose} size="4xl">
      <ModalOverlay />
      <ModalContent>
        <ModalHeader>{plugin.name} - Details</ModalHeader>
        <ModalCloseButton />
        <ModalBody>
          <Tabs>
            <TabList>
              <Tab>Overview</Tab>
              <Tab>Performance</Tab>
              <Tab>Configuration</Tab>
              <Tab>Logs</Tab>
            </TabList>
            
            <TabPanels>
              <TabPanel>
                <VStack spacing={6} align="stretch">
                  <SimpleGrid columns={2} spacing={6}>
                    <Box>
                      <Heading size="sm" mb={3}>Plugin Information</Heading>
                      <VStack align="start" spacing={2}>
                        <HStack>
                          <Text fontWeight="bold" w="120px">Name:</Text>
                          <Text>{plugin.name}</Text>
                        </HStack>
                        <HStack>
                          <Text fontWeight="bold" w="120px">Version:</Text>
                          <Badge>{plugin.version}</Badge>
                        </HStack>
                        <HStack>
                          <Text fontWeight="bold" w="120px">Author:</Text>
                          <Text>{plugin.author}</Text>
                        </HStack>
                        <HStack>
                          <Text fontWeight="bold" w="120px">Status:</Text>
                          <Badge colorScheme={
                            plugin.status === 'running' ? 'green' :
                            plugin.status === 'stopped' ? 'gray' : 'red'
                          }>
                            {plugin.status}
                          </Badge>
                        </HStack>
                      </VStack>
                    </Box>

                    <Box>
                      <Heading size="sm" mb={3}>Performance Metrics</Heading>
                      <VStack align="start" spacing={2}>
                        <HStack>
                          <Text fontWeight="bold" w="120px">Inferences:</Text>
                          <Text>{plugin.inference_count.toLocaleString()}</Text>
                        </HStack>
                        <HStack>
                          <Text fontWeight="bold" w="120px">Avg Latency:</Text>
                          <Text>{(plugin.average_inference_time * 1000).toFixed(1)}ms</Text>
                        </HStack>
                        <HStack>
                          <Text fontWeight="bold" w="120px">Current FPS:</Text>
                          <Text>{plugin.fps.toFixed(1)}</Text>
                        </HStack>
                        <HStack>
                          <Text fontWeight="bold" w="120px">Accuracy:</Text>
                          <Text>{(plugin.accuracy * 100).toFixed(1)}%</Text>
                        </HStack>
                      </VStack>
                    </Box>
                  </SimpleGrid>

                  <Box>
                    <Heading size="sm" mb={3}>Description</Heading>
                    <Text color="gray.600">{plugin.description}</Text>
                  </Box>

                  {plugin.last_error && (
                    <Alert status="error">
                      <AlertIcon />
                      <Box>
                        <Text fontWeight="bold">Last Error:</Text>
                        <Text fontSize="sm">{plugin.last_error}</Text>
                      </Box>
                    </Alert>
                  )}
                </VStack>
              </TabPanel>
              
              <TabPanel>
                <VStack spacing={6} align="stretch">
                  <SimpleGrid columns={3} spacing={4}>
                    <Box textAlign="center">
                      <CircularProgress value={plugin.fps * 3.33} color="blue.400" size="80px">
                        <CircularProgressLabel>{plugin.fps.toFixed(1)} FPS</CircularProgressLabel>
                      </CircularProgress>
                      <Text fontSize="sm" mt={2}>Frames per Second</Text>
                    </Box>
                    
                    <Box textAlign="center">
                      <CircularProgress value={plugin.gpu_usage} color="orange.400" size="80px">
                        <CircularProgressLabel>{plugin.gpu_usage}%</CircularProgressLabel>
                      </CircularProgress>
                      <Text fontSize="sm" mt={2}>GPU Usage</Text>
                    </Box>
                    
                    <Box textAlign="center">
                      <CircularProgress 
                        value={plugin.memory_usage * 10} 
                        color="green.400" 
                        size="80px"
                      >
                        <CircularProgressLabel>{plugin.memory_usage.toFixed(1)}GB</CircularProgressLabel>
                      </CircularProgress>
                      <Text fontSize="sm" mt={2}>Memory Usage</Text>
                    </Box>
                  </SimpleGrid>

                  {plugin.performance_history.length > 0 && (
                    <Box>
                      <Heading size="sm" mb={4}>Performance History (1 hour)</Heading>
                      <Box height="300px">
                        <ResponsiveContainer width="100%" height="100%">
                          <AreaChart data={plugin.performance_history}>
                            <CartesianGrid strokeDasharray="3 3" />
                            <XAxis dataKey="time" />
                            <YAxis yAxisId="left" />
                            <YAxis yAxisId="right" orientation="right" />
                            <RechartsTooltip />
                            <Area 
                              yAxisId="left" 
                              type="monotone" 
                              dataKey="fps" 
                              stroke="#1e88e5" 
                              fill="#1e88e5" 
                              fillOpacity={0.3}
                              name="FPS"
                            />
                            <Line 
                              yAxisId="right" 
                              type="monotone" 
                              dataKey="latency" 
                              stroke="#43a047" 
                              strokeWidth={2}
                              name="Latency (s)"
                            />
                          </AreaChart>
                        </ResponsiveContainer>
                      </Box>
                    </Box>
                  )}
                </VStack>
              </TabPanel>
              
              <TabPanel>
                <VStack spacing={4} align="stretch">
                  <Text fontSize="sm" color="gray.600">
                    Current plugin configuration (read-only view)
                  </Text>
                  <Code p={4} rounded="md" fontSize="sm" whiteSpace="pre-wrap">
                    {JSON.stringify(plugin.config, null, 2)}
                  </Code>
                </VStack>
              </TabPanel>
              
              <TabPanel>
                <VStack spacing={4} align="stretch">
                  <Text fontSize="sm" color="gray.600">
                    Recent plugin logs and error messages
                  </Text>
                  <Box 
                    bg={useColorModeValue('gray.50', 'gray.700')} 
                    p={4} 
                    rounded="md" 
                    maxH="300px" 
                    overflowY="auto"
                    fontFamily="mono"
                    fontSize="sm"
                  >
                    <Text color="green.500">[2024-01-15 10:30:15] INFO: Plugin initialized successfully</Text>
                    <Text color="blue.500">[2024-01-15 10:30:16] DEBUG: Loading model from {plugin.config.model_path || '/models/yolov8n.pt'}</Text>
                    <Text color="green.500">[2024-01-15 10:30:18] INFO: Model loaded, ready for inference</Text>
                    {plugin.status === 'running' && (
                      <>
                        <Text color="blue.500">[2024-01-15 10:30:20] DEBUG: Processing frame batch_size=1</Text>
                        <Text color="green.500">[2024-01-15 10:30:21] INFO: Inference completed in {(plugin.average_inference_time * 1000).toFixed(1)}ms</Text>
                      </>
                    )}
                    {plugin.error_count > 0 && (
                      <Text color="red.500">[2024-01-15 10:25:10] ERROR: {plugin.last_error}</Text>
                    )}
                  </Box>
                </VStack>
              </TabPanel>
            </TabPanels>
          </Tabs>
        </ModalBody>
        
        <ModalFooter>
          <Button variant="ghost" onClick={onClose}>Close</Button>
        </ModalFooter>
      </ModalContent>
    </Modal>
  );
};

export default function Plugins() {
  const [plugins, setPlugins] = useState(mockPlugins);
  const [selectedPlugin, setSelectedPlugin] = useState(null);
  const [configPlugin, setConfigPlugin] = useState(null);
  const [isDetailOpen, setIsDetailOpen] = useState(false);
  const [isConfigOpen, setIsConfigOpen] = useState(false);
  
  const toast = useToast();
  const cardBg = useColorModeValue('white', 'gray.800');
  const borderColor = useColorModeValue('gray.200', 'gray.600');

  const handleTogglePlugin = (pluginId) => {
    setPlugins(prevPlugins => 
      prevPlugins.map(plugin => 
        plugin.id === pluginId 
          ? { 
              ...plugin, 
              enabled: !plugin.enabled,
              status: !plugin.enabled ? 'running' : 'stopped',
              fps: !plugin.enabled ? 20 + Math.random() * 10 : 0,
              gpu_usage: !plugin.enabled ? 60 + Math.random() * 30 : 0,
            }
          : plugin
      )
    );
    
    const plugin = plugins.find(p => p.id === pluginId);
    toast({
      title: plugin?.enabled ? 'Plugin Stopped' : 'Plugin Started',
      description: `${plugin?.name} has been ${plugin?.enabled ? 'stopped' : 'started'}.`,
      status: 'info',
      duration: 3000,
    });
  };

  const handleViewDetails = (plugin) => {
    setSelectedPlugin(plugin);
    setIsDetailOpen(true);
  };

  const handleConfigurePlugin = (plugin) => {
    setConfigPlugin(plugin);
    setIsConfigOpen(true);
  };

  const handleSaveConfig = (pluginId, newConfig) => {
    setPlugins(prevPlugins => 
      prevPlugins.map(plugin => 
        plugin.id === pluginId 
          ? { ...plugin, config: newConfig }
          : plugin
      )
    );
    
    const plugin = plugins.find(p => p.id === pluginId);
    toast({
      title: 'Configuration Updated',
      description: `Configuration for ${plugin?.name} has been saved.`,
      status: 'success',
      duration: 3000,
    });
  };

  // Statistics
  const stats = {
    total: plugins.length,
    running: plugins.filter(p => p.status === 'running').length,
    errors: plugins.filter(p => p.status === 'error').length,
    totalInferences: plugins.reduce((sum, p) => sum + p.inference_count, 0),
  };

  const getStatusColor = (status) => {
    switch (status) {
      case 'running': return 'green';
      case 'stopped': return 'gray';
      case 'error': return 'red';
      default: return 'gray';
    }
  };

  return (
    <Layout>
      <VStack spacing={6} align="stretch">
        {/* Header */}
        <HStack justify="space-between" align="center">
          <Box>
            <Heading size="lg" mb={2}>Plugin Management</Heading>
            <Text color="gray.600">Monitor and configure AI inference plugins</Text>
          </Box>

          <HStack spacing={3}>
            <Button leftIcon={<FiRefreshCw />} variant="outline" size="sm">
              Refresh
            </Button>
            <Button leftIcon={<FiDownload />} variant="outline" size="sm">
              Install Plugin
            </Button>
          </HStack>
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
            <StatLabel>Total Plugins</StatLabel>
            <StatNumber>{stats.total}</StatNumber>
            <StatHelpText>Installed plugins</StatHelpText>
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
            <StatLabel>Running</StatLabel>
            <StatNumber>{stats.running}</StatNumber>
            <StatHelpText>Active plugins</StatHelpText>
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
            <StatLabel>Errors</StatLabel>
            <StatNumber>{stats.errors}</StatNumber>
            <StatHelpText>Plugins with issues</StatHelpText>
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
            <StatLabel>Total Inferences</StatLabel>
            <StatNumber>{stats.totalInferences.toLocaleString()}</StatNumber>
            <StatHelpText>All time</StatHelpText>
          </Stat>
        </SimpleGrid>

        {/* Plugins Grid */}
        <SimpleGrid columns={{ base: 1, lg: 2, xl: 3 }} spacing={6}>
          {plugins.map((plugin) => (
            <Box
              key={plugin.id}
              bg={cardBg}
              rounded="lg"
              border="1px"
              borderColor={borderColor}
              p={6}
              shadow="sm"
            >
              <VStack align="stretch" spacing={4}>
                {/* Header */}
                <HStack justify="space-between" align="start">
                  <VStack align="start" spacing={1}>
                    <Heading size="md">{plugin.name}</Heading>
                    <HStack>
                      <Badge colorScheme={getStatusColor(plugin.status)} variant="solid">
                        {plugin.status}
                      </Badge>
                      <Badge variant="outline">v{plugin.version}</Badge>
                    </HStack>
                  </VStack>
                  
                  <Switch
                    isChecked={plugin.enabled}
                    onChange={() => handleTogglePlugin(plugin.id)}
                    colorScheme="green"
                  />
                </HStack>

                {/* Description */}
                <Text fontSize="sm" color="gray.600" noOfLines={2}>
                  {plugin.description}
                </Text>

                {/* Metrics */}
                <SimpleGrid columns={2} spacing={4}>
                  <Box>
                    <Text fontSize="xs" color="gray.500">INFERENCES</Text>
                    <Text fontWeight="bold">{plugin.inference_count.toLocaleString()}</Text>
                  </Box>
                  <Box>
                    <Text fontSize="xs" color="gray.500">AVG LATENCY</Text>
                    <Text fontWeight="bold">{(plugin.average_inference_time * 1000).toFixed(1)}ms</Text>
                  </Box>
                  <Box>
                    <Text fontSize="xs" color="gray.500">FPS</Text>
                    <Text fontWeight="bold">{plugin.fps.toFixed(1)}</Text>
                  </Box>
                  <Box>
                    <Text fontSize="xs" color="gray.500">ACCURACY</Text>
                    <Text fontWeight="bold">{(plugin.accuracy * 100).toFixed(1)}%</Text>
                  </Box>
                </SimpleGrid>

                {/* Resource Usage */}
                {plugin.status === 'running' && (
                  <VStack spacing={2}>
                    <HStack justify="space-between" w="full">
                      <Text fontSize="sm">GPU Usage</Text>
                      <Text fontSize="sm">{plugin.gpu_usage}%</Text>
                    </HStack>
                    <Progress value={plugin.gpu_usage} colorScheme="orange" size="sm" w="full" />
                    
                    <HStack justify="space-between" w="full">
                      <Text fontSize="sm">Memory</Text>
                      <Text fontSize="sm">{plugin.memory_usage.toFixed(1)} GB</Text>
                    </HStack>
                    <Progress value={plugin.memory_usage * 10} colorScheme="blue" size="sm" w="full" />
                  </VStack>
                )}

                {/* Error Display */}
                {plugin.error_count > 0 && (
                  <Alert status="warning" size="sm">
                    <AlertIcon />
                    <Text fontSize="xs">{plugin.error_count} error{plugin.error_count !== 1 ? 's' : ''}</Text>
                  </Alert>
                )}

                {/* Actions */}
                <HStack spacing={2}>
                  <Button
                    size="sm"
                    leftIcon={<FiEye />}
                    variant="outline"
                    onClick={() => handleViewDetails(plugin)}
                    flex={1}
                  >
                    Details
                  </Button>
                  
                  <Button
                    size="sm"
                    leftIcon={<FiSettings />}
                    variant="outline"
                    onClick={() => handleConfigurePlugin(plugin)}
                    flex={1}
                  >
                    Configure
                  </Button>
                </HStack>
              </VStack>
            </Box>
          ))}
        </SimpleGrid>

        {/* Plugin Detail Modal */}
        <PluginDetailModal
          plugin={selectedPlugin}
          isOpen={isDetailOpen}
          onClose={() => setIsDetailOpen(false)}
        />

        {/* Plugin Config Modal */}
        <PluginConfigModal
          plugin={configPlugin}
          isOpen={isConfigOpen}
          onClose={() => setIsConfigOpen(false)}
          onSave={handleSaveConfig}
        />
      </VStack>
    </Layout>
  );
}