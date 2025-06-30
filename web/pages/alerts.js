import { useEffect, useState } from 'react';
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
  InputGroup,
  InputLeftElement,
  Alert,
  AlertIcon,
  AlertTitle,
  AlertDescription,
  useToast,
  Modal,
  ModalOverlay,
  ModalContent,
  ModalHeader,
  ModalFooter,
  ModalBody,
  ModalCloseButton,
  useDisclosure,
} from '@chakra-ui/react';
import {
  FiAlertTriangle,
  FiCheckCircle,
  FiXCircle,
  FiSearch,
  FiFilter,
  FiEye,
  FiTrash2,
} from 'react-icons/fi';
import { format } from 'date-fns';
import Layout from '../components/Layout';

export default function Alerts() {
  const [alerts, setAlerts] = useState([]);
  const [filteredAlerts, setFilteredAlerts] = useState([]);
  const [filter, setFilter] = useState('all');
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedAlert, setSelectedAlert] = useState(null);
  const [isConnected, setIsConnected] = useState(false);
  
  const { isOpen, onOpen, onClose } = useDisclosure();
  const toast = useToast();
  
  const cardBg = useColorModeValue('white', 'gray.800');
  const borderColor = useColorModeValue('gray.200', 'gray.600');

  useEffect(() => {
    const evt = new EventSource('http://localhost:5000/alerts/stream');
    
    evt.onopen = () => {
      setIsConnected(true);
    };
    
    evt.onmessage = (e) => {
      try {
        const newAlert = JSON.parse(e.data);
        setAlerts((prevAlerts) => {
          const updated = [{
            ...newAlert,
            id: Date.now(),
            timestamp: new Date(newAlert.timestamp * 1000),
            acknowledged: false,
            severity: getSeverity(newAlert),
          }, ...prevAlerts.slice(0, 99)];
          return updated;
        });
        
        // Show toast notification for new alert
        toast({
          title: 'New Alert',
          description: newAlert.description,
          status: 'warning',
          duration: 5000,
          isClosable: true,
        });
      } catch (error) {
        console.error('Error parsing alert data:', error);
      }
    };
    
    evt.onerror = () => {
      setIsConnected(false);
    };
    
    return () => {
      evt.close();
      setIsConnected(false);
    };
  }, [toast]);

  // Filter and search alerts
  useEffect(() => {
    let filtered = alerts;
    
    // Filter by status
    if (filter === 'active') {
      filtered = filtered.filter(alert => !alert.acknowledged);
    } else if (filter === 'acknowledged') {
      filtered = filtered.filter(alert => alert.acknowledged);
    }
    
    // Search filter
    if (searchTerm) {
      filtered = filtered.filter(alert => 
        alert.description.toLowerCase().includes(searchTerm.toLowerCase()) ||
        alert.sensor_id.toLowerCase().includes(searchTerm.toLowerCase()) ||
        alert.plugin.toLowerCase().includes(searchTerm.toLowerCase())
      );
    }
    
    setFilteredAlerts(filtered);
  }, [alerts, filter, searchTerm]);

  const getSeverity = (alert) => {
    // Determine severity based on alert content
    const desc = alert.description.toLowerCase();
    if (desc.includes('error') || desc.includes('failed') || desc.includes('critical')) {
      return 'high';
    } else if (desc.includes('warning') || desc.includes('anomaly')) {
      return 'medium';
    }
    return 'low';
  };

  const getSeverityColor = (severity) => {
    switch (severity) {
      case 'high': return 'red';
      case 'medium': return 'orange';
      case 'low': return 'yellow';
      default: return 'gray';
    }
  };

  const acknowledgeAlert = (alertId) => {
    setAlerts(prevAlerts => 
      prevAlerts.map(alert => 
        alert.id === alertId 
          ? { ...alert, acknowledged: true, acknowledgedAt: new Date() }
          : alert
      )
    );
    
    toast({
      title: 'Alert Acknowledged',
      status: 'success',
      duration: 2000,
    });
  };

  const deleteAlert = (alertId) => {
    setAlerts(prevAlerts => prevAlerts.filter(alert => alert.id !== alertId));
    
    toast({
      title: 'Alert Deleted',
      status: 'info',
      duration: 2000,
    });
  };

  const viewAlertDetails = (alert) => {
    setSelectedAlert(alert);
    onOpen();
  };

  const activeAlerts = alerts.filter(alert => !alert.acknowledged);
  const acknowledgedAlerts = alerts.filter(alert => alert.acknowledged);

  return (
    <Layout>
      <VStack spacing={8} align="stretch">
        {/* Header */}
        <HStack justify="space-between" align="center">
          <Box>
            <Heading size="lg" mb={2}>Alert Management</Heading>
            <HStack>
              <Text color="gray.600">Monitor and manage system alerts</Text>
              <Badge colorScheme={isConnected ? 'green' : 'red'} variant="subtle">
                {isConnected ? 'Connected' : 'Disconnected'}
              </Badge>
            </HStack>
          </Box>
          
          <HStack>
            <Badge colorScheme="red" variant="subtle" fontSize="md" px={3} py={1}>
              {activeAlerts.length} Active
            </Badge>
            <Badge colorScheme="green" variant="subtle" fontSize="md" px={3} py={1}>
              {acknowledgedAlerts.length} Resolved
            </Badge>
          </HStack>
        </HStack>

        {/* Active Alerts Summary */}
        {activeAlerts.length > 0 && (
          <Alert status="warning" rounded="lg">
            <AlertIcon />
            <Box>
              <AlertTitle>Active Alerts!</AlertTitle>
              <AlertDescription>
                You have {activeAlerts.length} unacknowledged alert{activeAlerts.length !== 1 ? 's' : ''} requiring attention.
              </AlertDescription>
            </Box>
          </Alert>
        )}

        {/* Filters and Search */}
        <HStack spacing={4}>
          <InputGroup maxW="300px">
            <InputLeftElement pointerEvents="none">
              <FiSearch color="gray.300" />
            </InputLeftElement>
            <Input
              placeholder="Search alerts..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
            />
          </InputGroup>
          
          <Select value={filter} onChange={(e) => setFilter(e.target.value)} maxW="200px">
            <option value="all">All Alerts</option>
            <option value="active">Active Only</option>
            <option value="acknowledged">Acknowledged Only</option>
          </Select>
        </HStack>

        {/* Alerts Table */}
        <Box bg={cardBg} rounded="lg" border="1px" borderColor={borderColor} overflow="hidden">
          <TableContainer>
            <Table variant="simple">
              <Thead bg={useColorModeValue('gray.50', 'gray.700')}>
                <Tr>
                  <Th>Severity</Th>
                  <Th>Description</Th>
                  <Th>Source</Th>
                  <Th>Plugin</Th>
                  <Th>Time</Th>
                  <Th>Status</Th>
                  <Th>Actions</Th>
                </Tr>
              </Thead>
              <Tbody>
                {filteredAlerts.map((alert) => (
                  <Tr key={alert.id}>
                    <Td>
                      <Badge colorScheme={getSeverityColor(alert.severity)} variant="solid">
                        {alert.severity.toUpperCase()}
                      </Badge>
                    </Td>
                    <Td maxW="300px">
                      <Text isTruncated>{alert.description}</Text>
                    </Td>
                    <Td>
                      <Badge colorScheme="blue" variant="subtle">
                        {alert.sensor_id}
                      </Badge>
                    </Td>
                    <Td>{alert.plugin}</Td>
                    <Td>
                      <Text fontSize="sm">
                        {format(alert.timestamp, 'MMM dd, HH:mm:ss')}
                      </Text>
                    </Td>
                    <Td>
                      {alert.acknowledged ? (
                        <Badge colorScheme="green" variant="subtle">
                          <FiCheckCircle style={{ marginRight: '4px' }} />
                          Acknowledged
                        </Badge>
                      ) : (
                        <Badge colorScheme="red" variant="subtle">
                          <FiAlertTriangle style={{ marginRight: '4px' }} />
                          Active
                        </Badge>
                      )}
                    </Td>
                    <Td>
                      <HStack spacing={2}>
                        <Tooltip label="View Details">
                          <IconButton
                            size="sm"
                            icon={<FiEye />}
                            onClick={() => viewAlertDetails(alert)}
                            variant="ghost"
                          />
                        </Tooltip>
                        
                        {!alert.acknowledged && (
                          <Tooltip label="Acknowledge">
                            <IconButton
                              size="sm"
                              icon={<FiCheckCircle />}
                              onClick={() => acknowledgeAlert(alert.id)}
                              colorScheme="green"
                              variant="ghost"
                            />
                          </Tooltip>
                        )}
                        
                        <Tooltip label="Delete">
                          <IconButton
                            size="sm"
                            icon={<FiTrash2 />}
                            onClick={() => deleteAlert(alert.id)}
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
          
          {filteredAlerts.length === 0 && (
            <Box p={8} textAlign="center">
              <Text color="gray.500">
                {alerts.length === 0 
                  ? 'No alerts received yet' 
                  : 'No alerts match your current filters'
                }
              </Text>
            </Box>
          )}
        </Box>

        {/* Alert Details Modal */}
        <Modal isOpen={isOpen} onClose={onClose} size="lg">
          <ModalOverlay />
          <ModalContent>
            <ModalHeader>Alert Details</ModalHeader>
            <ModalCloseButton />
            <ModalBody>
              {selectedAlert && (
                <VStack align="stretch" spacing={4}>
                  <HStack justify="space-between">
                    <Text fontWeight="bold">Severity:</Text>
                    <Badge colorScheme={getSeverityColor(selectedAlert.severity)} variant="solid">
                      {selectedAlert.severity.toUpperCase()}
                    </Badge>
                  </HStack>
                  
                  <Box>
                    <Text fontWeight="bold" mb={2}>Description:</Text>
                    <Text>{selectedAlert.description}</Text>
                  </Box>
                  
                  <HStack justify="space-between">
                    <Text fontWeight="bold">Source Sensor:</Text>
                    <Badge colorScheme="blue">{selectedAlert.sensor_id}</Badge>
                  </HStack>
                  
                  <HStack justify="space-between">
                    <Text fontWeight="bold">Plugin:</Text>
                    <Text>{selectedAlert.plugin}</Text>
                  </HStack>
                  
                  <HStack justify="space-between">
                    <Text fontWeight="bold">Timestamp:</Text>
                    <Text>{format(selectedAlert.timestamp, 'MMM dd, yyyy HH:mm:ss')}</Text>
                  </HStack>
                  
                  {selectedAlert.metadata && Object.keys(selectedAlert.metadata).length > 0 && (
                    <Box>
                      <Text fontWeight="bold" mb={2}>Metadata:</Text>
                      <Box bg={useColorModeValue('gray.50', 'gray.700')} p={3} rounded="md">
                        {Object.entries(selectedAlert.metadata).map(([key, value]) => (
                          <HStack key={key} justify="space-between">
                            <Text fontSize="sm" fontWeight="medium">{key}:</Text>
                            <Text fontSize="sm">{value}</Text>
                          </HStack>
                        ))}
                      </Box>
                    </Box>
                  )}
                </VStack>
              )}
            </ModalBody>
            
            <ModalFooter>
              {selectedAlert && !selectedAlert.acknowledged && (
                <Button 
                  colorScheme="green" 
                  mr={3} 
                  onClick={() => {
                    acknowledgeAlert(selectedAlert.id);
                    onClose();
                  }}
                >
                  Acknowledge
                </Button>
              )}
              <Button variant="ghost" onClick={onClose}>Close</Button>
            </ModalFooter>
          </ModalContent>
        </Modal>
      </VStack>
    </Layout>
  );
}
