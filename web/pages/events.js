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
  InputGroup,
  InputLeftElement,
  RangeSlider,
  RangeSliderTrack,
  RangeSliderFilledTrack,
  RangeSliderThumb,
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
  Menu,
  MenuButton,
  MenuList,
  MenuItem,
  useToast,
  Flex,
  Spacer,
} from '@chakra-ui/react';
import {
  FiSearch,
  FiFilter,
  FiEye,
  FiDownload,
  FiRefreshCw,
  FiCalendar,
  FiCamera,
  FiCpu,
  FiMoreVertical,
  FiPlay,
  FiPause,
} from 'react-icons/fi';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip as RechartsTooltip, ResponsiveContainer, BarChart, Bar } from 'recharts';
import { format, subHours, subDays } from 'date-fns';
import Layout from '../components/Layout';

// Mock events data - replace with real API calls
const generateMockEvents = () => {
  const plugins = ['yolov8', 'yolov5', 'custom_detector'];
  const sensors = ['usb_camera_0', 'csi_camera', 'ip_camera_1', 'ip_camera_2'];
  const classes = ['person', 'vehicle', 'animal', 'package', 'anomaly'];
  
  return Array.from({ length: 150 }, (_, i) => ({
    id: `event_${i}`,
    timestamp: new Date(Date.now() - Math.random() * 7 * 24 * 60 * 60 * 1000), // Last 7 days
    sensor_id: sensors[Math.floor(Math.random() * sensors.length)],
    plugin: plugins[Math.floor(Math.random() * plugins.length)],
    event_type: 'detection',
    class: classes[Math.floor(Math.random() * classes.length)],
    confidence: Math.random() * 0.5 + 0.5, // 0.5 - 1.0
    bbox: [
      Math.floor(Math.random() * 500),
      Math.floor(Math.random() * 300),
      Math.floor(Math.random() * 500) + 500,
      Math.floor(Math.random() * 300) + 300,
    ],
    metadata: {
      inference_time: Math.random() * 0.1 + 0.02,
      model_version: '1.0.0',
      frame_id: Math.floor(Math.random() * 10000),
    },
  }));
};

const EventDetailModal = ({ event, isOpen, onClose }) => {
  if (!event) return null;

  return (
    <Modal isOpen={isOpen} onClose={onClose} size="xl">
      <ModalOverlay />
      <ModalContent>
        <ModalHeader>Event Details</ModalHeader>
        <ModalCloseButton />
        <ModalBody>
          <VStack align="stretch" spacing={4}>
            <SimpleGrid columns={2} spacing={4}>
              <Box>
                <Text fontWeight="bold" color="gray.600" fontSize="sm">EVENT ID</Text>
                <Text>{event.id}</Text>
              </Box>
              <Box>
                <Text fontWeight="bold" color="gray.600" fontSize="sm">TIMESTAMP</Text>
                <Text>{format(event.timestamp, 'MMM dd, yyyy HH:mm:ss')}</Text>
              </Box>
              <Box>
                <Text fontWeight="bold" color="gray.600" fontSize="sm">SENSOR</Text>
                <Badge colorScheme="blue" variant="subtle">{event.sensor_id}</Badge>
              </Box>
              <Box>
                <Text fontWeight="bold" color="gray.600" fontSize="sm">PLUGIN</Text>
                <Text>{event.plugin}</Text>
              </Box>
              <Box>
                <Text fontWeight="bold" color="gray.600" fontSize="sm">DETECTION CLASS</Text>
                <Badge colorScheme="green" variant="solid">{event.class}</Badge>
              </Box>
              <Box>
                <Text fontWeight="bold" color="gray.600" fontSize="sm">CONFIDENCE</Text>
                <Badge colorScheme={event.confidence > 0.8 ? 'green' : 'yellow'} variant="outline">
                  {(event.confidence * 100).toFixed(1)}%
                </Badge>
              </Box>
            </SimpleGrid>

            <Box>
              <Text fontWeight="bold" color="gray.600" fontSize="sm" mb={2}>BOUNDING BOX</Text>
              <HStack spacing={4}>
                <Text fontSize="sm">X: {event.bbox[0]}</Text>
                <Text fontSize="sm">Y: {event.bbox[1]}</Text>
                <Text fontSize="sm">Width: {event.bbox[2] - event.bbox[0]}</Text>
                <Text fontSize="sm">Height: {event.bbox[3] - event.bbox[1]}</Text>
              </HStack>
            </Box>

            <Box>
              <Text fontWeight="bold" color="gray.600" fontSize="sm" mb={2}>METADATA</Text>
              <Box bg={useColorModeValue('gray.50', 'gray.700')} p={3} rounded="md">
                <SimpleGrid columns={2} spacing={2}>
                  <Text fontSize="sm"><strong>Inference Time:</strong> {(event.metadata.inference_time * 1000).toFixed(1)}ms</Text>
                  <Text fontSize="sm"><strong>Model Version:</strong> {event.metadata.model_version}</Text>
                  <Text fontSize="sm"><strong>Frame ID:</strong> {event.metadata.frame_id}</Text>
                  <Text fontSize="sm"><strong>Event Type:</strong> {event.event_type}</Text>
                </SimpleGrid>
              </Box>
            </Box>
          </VStack>
        </ModalBody>
        <ModalFooter>
          <Button variant="ghost" onClick={onClose}>Close</Button>
        </ModalFooter>
      </ModalContent>
    </Modal>
  );
};

export default function Events() {
  const [events, setEvents] = useState([]);
  const [filteredEvents, setFilteredEvents] = useState([]);
  const [selectedEvent, setSelectedEvent] = useState(null);
  const [filters, setFilters] = useState({
    search: '',
    sensor: 'all',
    plugin: 'all',
    class: 'all',
    timeRange: '24h',
    confidenceRange: [50, 100],
  });
  const [isStreaming, setIsStreaming] = useState(false);

  const { isOpen, onOpen, onClose } = useDisclosure();
  const toast = useToast();
  const cardBg = useColorModeValue('white', 'gray.800');
  const borderColor = useColorModeValue('gray.200', 'gray.600');

  // Initialize with mock data
  useEffect(() => {
    const mockEvents = generateMockEvents();
    setEvents(mockEvents);
  }, []);

  // Simulate real-time event streaming
  useEffect(() => {
    if (!isStreaming) return;

    const interval = setInterval(() => {
      const newEvent = {
        id: `event_${Date.now()}`,
        timestamp: new Date(),
        sensor_id: ['usb_camera_0', 'csi_camera'][Math.floor(Math.random() * 2)],
        plugin: 'yolov8',
        event_type: 'detection',
        class: ['person', 'vehicle'][Math.floor(Math.random() * 2)],
        confidence: Math.random() * 0.5 + 0.5,
        bbox: [
          Math.floor(Math.random() * 500),
          Math.floor(Math.random() * 300),
          Math.floor(Math.random() * 500) + 500,
          Math.floor(Math.random() * 300) + 300,
        ],
        metadata: {
          inference_time: Math.random() * 0.1 + 0.02,
          model_version: '1.0.0',
          frame_id: Math.floor(Math.random() * 10000),
        },
      };

      setEvents(prev => [newEvent, ...prev.slice(0, 199)]);
    }, 2000 + Math.random() * 3000); // Random interval 2-5 seconds

    return () => clearInterval(interval);
  }, [isStreaming]);

  // Apply filters
  useEffect(() => {
    let filtered = [...events];

    // Time range filter
    const now = new Date();
    const timeRanges = {
      '1h': subHours(now, 1),
      '6h': subHours(now, 6),
      '24h': subHours(now, 24),
      '7d': subDays(now, 7),
    };
    
    if (filters.timeRange !== 'all') {
      const cutoff = timeRanges[filters.timeRange];
      filtered = filtered.filter(event => event.timestamp >= cutoff);
    }

    // Search filter
    if (filters.search) {
      filtered = filtered.filter(event =>
        event.id.toLowerCase().includes(filters.search.toLowerCase()) ||
        event.sensor_id.toLowerCase().includes(filters.search.toLowerCase()) ||
        event.class.toLowerCase().includes(filters.search.toLowerCase()) ||
        event.plugin.toLowerCase().includes(filters.search.toLowerCase())
      );
    }

    // Sensor filter
    if (filters.sensor !== 'all') {
      filtered = filtered.filter(event => event.sensor_id === filters.sensor);
    }

    // Plugin filter
    if (filters.plugin !== 'all') {
      filtered = filtered.filter(event => event.plugin === filters.plugin);
    }

    // Class filter
    if (filters.class !== 'all') {
      filtered = filtered.filter(event => event.class === filters.class);
    }

    // Confidence filter
    filtered = filtered.filter(event => {
      const confidence = event.confidence * 100;
      return confidence >= filters.confidenceRange[0] && confidence <= filters.confidenceRange[1];
    });

    // Sort by timestamp (newest first)
    filtered.sort((a, b) => new Date(b.timestamp) - new Date(a.timestamp));

    setFilteredEvents(filtered);
  }, [events, filters]);

  const handleViewEvent = (event) => {
    setSelectedEvent(event);
    onOpen();
  };

  const handleExportEvents = () => {
    const csvContent = [
      ['ID', 'Timestamp', 'Sensor', 'Plugin', 'Class', 'Confidence', 'Inference Time (ms)'].join(','),
      ...filteredEvents.map(event => [
        event.id,
        event.timestamp.toISOString(),
        event.sensor_id,
        event.plugin,
        event.class,
        (event.confidence * 100).toFixed(1),
        (event.metadata.inference_time * 1000).toFixed(1),
      ].join(','))
    ].join('\n');

    const blob = new Blob([csvContent], { type: 'text/csv' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `vargard_events_${format(new Date(), 'yyyy-MM-dd_HH-mm')}.csv`;
    a.click();
    window.URL.revokeObjectURL(url);

    toast({
      title: 'Export Complete',
      description: `Exported ${filteredEvents.length} events to CSV`,
      status: 'success',
      duration: 3000,
    });
  };

  // Statistics
  const stats = {
    total: filteredEvents.length,
    highConfidence: filteredEvents.filter(e => e.confidence > 0.8).length,
    uniqueSensors: new Set(filteredEvents.map(e => e.sensor_id)).size,
    avgConfidence: filteredEvents.length > 0 
      ? (filteredEvents.reduce((sum, e) => sum + e.confidence, 0) / filteredEvents.length * 100).toFixed(1)
      : 0,
  };

  // Chart data
  const hourlyData = Array.from({ length: 24 }, (_, i) => {
    const hour = new Date();
    hour.setHours(hour.getHours() - (23 - i), 0, 0, 0);
    const hourEvents = filteredEvents.filter(event => 
      event.timestamp.getHours() === hour.getHours() &&
      event.timestamp.getDate() === hour.getDate()
    );
    return {
      hour: format(hour, 'HH:mm'),
      events: hourEvents.length,
    };
  });

  const classData = Object.entries(
    filteredEvents.reduce((acc, event) => {
      acc[event.class] = (acc[event.class] || 0) + 1;
      return acc;
    }, {})
  ).map(([name, count]) => ({ name, count }));

  // Get unique values for filters
  const uniqueSensors = [...new Set(events.map(e => e.sensor_id))];
  const uniquePlugins = [...new Set(events.map(e => e.plugin))];
  const uniqueClasses = [...new Set(events.map(e => e.class))];

  return (
    <Layout>
      <VStack spacing={6} align="stretch">
        {/* Header */}
        <HStack justify="space-between" align="center">
          <Box>
            <Heading size="lg" mb={2}>Inference Events</Heading>
            <HStack>
              <Text color="gray.600">Real-time AI inference monitoring</Text>
              <Badge colorScheme={isStreaming ? 'green' : 'gray'} variant="subtle">
                {isStreaming ? 'Live' : 'Paused'}
              </Badge>
            </HStack>
          </Box>

          <HStack spacing={3}>
            <Button
              leftIcon={isStreaming ? <FiPause /> : <FiPlay />}
              colorScheme={isStreaming ? 'red' : 'green'}
              variant="outline"
              size="sm"
              onClick={() => setIsStreaming(!isStreaming)}
            >
              {isStreaming ? 'Pause' : 'Start'} Stream
            </Button>
            
            <Button
              leftIcon={<FiDownload />}
              variant="outline"
              size="sm"
              onClick={handleExportEvents}
              isDisabled={filteredEvents.length === 0}
            >
              Export CSV
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
            <StatLabel>Total Events</StatLabel>
            <StatNumber>{stats.total}</StatNumber>
            <StatHelpText>In selected time range</StatHelpText>
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
            <StatLabel>High Confidence</StatLabel>
            <StatNumber>{stats.highConfidence}</StatNumber>
                         <StatHelpText>&gt;80% confidence</StatHelpText>
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
            <StatLabel>Active Sensors</StatLabel>
            <StatNumber>{stats.uniqueSensors}</StatNumber>
            <StatHelpText>Contributing data</StatHelpText>
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
            <StatLabel>Avg Confidence</StatLabel>
            <StatNumber>{stats.avgConfidence}%</StatNumber>
            <StatHelpText>Across all events</StatHelpText>
          </Stat>
        </SimpleGrid>

        {/* Charts */}
        <SimpleGrid columns={{ base: 1, lg: 2 }} spacing={6}>
          <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
            <Heading size="md" mb={4}>Events Over Time (24h)</Heading>
            <Box height="250px">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={hourlyData}>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis dataKey="hour" />
                  <YAxis />
                  <RechartsTooltip />
                  <Line type="monotone" dataKey="events" stroke="#1e88e5" strokeWidth={2} />
                </LineChart>
              </ResponsiveContainer>
            </Box>
          </Box>

          <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
            <Heading size="md" mb={4}>Detection Classes</Heading>
            <Box height="250px">
              <ResponsiveContainer width="100%" height="100%">
                <BarChart data={classData}>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis dataKey="name" />
                  <YAxis />
                  <RechartsTooltip />
                  <Bar dataKey="count" fill="#43a047" />
                </BarChart>
              </ResponsiveContainer>
            </Box>
          </Box>
        </SimpleGrid>

        {/* Filters */}
        <Box bg={cardBg} p={4} rounded="lg" border="1px" borderColor={borderColor}>
          <Flex wrap="wrap" gap={4} align="end">
            <Box minW="200px">
              <Text fontSize="sm" mb={1}>Search</Text>
              <InputGroup>
                <InputLeftElement pointerEvents="none">
                  <FiSearch color="gray.300" />
                </InputLeftElement>
                <Input
                  placeholder="Search events..."
                  value={filters.search}
                  onChange={(e) => setFilters(prev => ({ ...prev, search: e.target.value }))}
                />
              </InputGroup>
            </Box>

            <Box minW="120px">
              <Text fontSize="sm" mb={1}>Time Range</Text>
              <Select
                value={filters.timeRange}
                onChange={(e) => setFilters(prev => ({ ...prev, timeRange: e.target.value }))}
              >
                <option value="1h">Last Hour</option>
                <option value="6h">Last 6 Hours</option>
                <option value="24h">Last 24 Hours</option>
                <option value="7d">Last 7 Days</option>
              </Select>
            </Box>

            <Box minW="120px">
              <Text fontSize="sm" mb={1}>Sensor</Text>
              <Select
                value={filters.sensor}
                onChange={(e) => setFilters(prev => ({ ...prev, sensor: e.target.value }))}
              >
                <option value="all">All Sensors</option>
                {uniqueSensors.map(sensor => (
                  <option key={sensor} value={sensor}>{sensor}</option>
                ))}
              </Select>
            </Box>

            <Box minW="120px">
              <Text fontSize="sm" mb={1}>Plugin</Text>
              <Select
                value={filters.plugin}
                onChange={(e) => setFilters(prev => ({ ...prev, plugin: e.target.value }))}
              >
                <option value="all">All Plugins</option>
                {uniquePlugins.map(plugin => (
                  <option key={plugin} value={plugin}>{plugin}</option>
                ))}
              </Select>
            </Box>

            <Box minW="120px">
              <Text fontSize="sm" mb={1}>Class</Text>
              <Select
                value={filters.class}
                onChange={(e) => setFilters(prev => ({ ...prev, class: e.target.value }))}
              >
                <option value="all">All Classes</option>
                {uniqueClasses.map(cls => (
                  <option key={cls} value={cls}>{cls}</option>
                ))}
              </Select>
            </Box>

            <Box minW="200px">
              <Text fontSize="sm" mb={1}>Confidence Range: {filters.confidenceRange[0]}% - {filters.confidenceRange[1]}%</Text>
              <RangeSlider
                value={filters.confidenceRange}
                onChange={(val) => setFilters(prev => ({ ...prev, confidenceRange: val }))}
                min={0}
                max={100}
                step={5}
              >
                <RangeSliderTrack>
                  <RangeSliderFilledTrack />
                </RangeSliderTrack>
                <RangeSliderThumb index={0} />
                <RangeSliderThumb index={1} />
              </RangeSlider>
            </Box>
          </Flex>
        </Box>

        {/* Events Table */}
        <Box bg={cardBg} rounded="lg" border="1px" borderColor={borderColor} overflow="hidden">
          <TableContainer>
            <Table variant="simple">
              <Thead bg={useColorModeValue('gray.50', 'gray.700')}>
                <Tr>
                  <Th>Timestamp</Th>
                  <Th>Sensor</Th>
                  <Th>Plugin</Th>
                  <Th>Class</Th>
                  <Th>Confidence</Th>
                  <Th>Inference Time</Th>
                  <Th>Actions</Th>
                </Tr>
              </Thead>
              <Tbody>
                {filteredEvents.slice(0, 50).map((event) => (
                  <Tr key={event.id}>
                    <Td>
                      <VStack align="start" spacing={0}>
                        <Text fontSize="sm">{format(event.timestamp, 'MMM dd, HH:mm:ss')}</Text>
                        <Text fontSize="xs" color="gray.500">{event.id}</Text>
                      </VStack>
                    </Td>
                    <Td>
                      <Badge colorScheme="blue" variant="subtle">
                        {event.sensor_id}
                      </Badge>
                    </Td>
                    <Td>{event.plugin}</Td>
                    <Td>
                      <Badge colorScheme="green" variant="solid">
                        {event.class}
                      </Badge>
                    </Td>
                    <Td>
                      <Badge 
                        colorScheme={event.confidence > 0.8 ? 'green' : 'yellow'}
                        variant="outline"
                      >
                        {(event.confidence * 100).toFixed(1)}%
                      </Badge>
                    </Td>
                    <Td>
                      <Text fontSize="sm">
                        {(event.metadata.inference_time * 1000).toFixed(1)}ms
                      </Text>
                    </Td>
                    <Td>
                      <Tooltip label="View Details">
                        <IconButton
                          size="sm"
                          icon={<FiEye />}
                          onClick={() => handleViewEvent(event)}
                          variant="ghost"
                        />
                      </Tooltip>
                    </Td>
                  </Tr>
                ))}
              </Tbody>
            </Table>
          </TableContainer>
          
          {filteredEvents.length === 0 && (
            <Box p={8} textAlign="center">
              <Text color="gray.500">No events match your current filters</Text>
            </Box>
          )}
          
          {filteredEvents.length > 50 && (
            <Box p={4} textAlign="center" borderTop="1px" borderColor={borderColor}>
              <Text color="gray.500" fontSize="sm">
                Showing first 50 of {filteredEvents.length} events. Use filters to narrow results.
              </Text>
            </Box>
          )}
        </Box>

        {/* Event Detail Modal */}
        <EventDetailModal
          event={selectedEvent}
          isOpen={isOpen}
          onClose={onClose}
        />
      </VStack>
    </Layout>
  );
}