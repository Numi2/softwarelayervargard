import {
  Box,
  SimpleGrid,
  Stat,
  StatLabel,
  StatNumber,
  StatHelpText,
  StatArrow,
  useColorModeValue,
  Heading,
  Text,
  VStack,
  HStack,
  Icon,
  Progress,
  Badge,
  Table,
  Thead,
  Tbody,
  Tr,
  Th,
  Td,
  TableContainer,
  Flex,
} from '@chakra-ui/react';
import { 
  FiCamera, 
  FiCpu, 
  FiAlertTriangle, 
  FiActivity,
  FiCheckCircle,
  FiXCircle 
} from 'react-icons/fi';
import { useQuery } from 'react-query';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, PieChart, Pie, Cell } from 'recharts';
import { format } from 'date-fns';
import Layout from '../components/Layout';

// Mock data - replace with real API calls
const mockSystemStats = {
  activeSensors: 3,
  totalAlerts: 12,
  avgFPS: 28.5,
  systemUptime: '2d 14h 32m',
  cpuUsage: 45,
  memoryUsage: 62,
  gpuUsage: 78,
};

const mockRecentEvents = [
  { id: 1, sensor: 'usb_camera_0', plugin: 'yolov8', event: 'person detected', confidence: 0.87, time: new Date() },
  { id: 2, sensor: 'csi_camera', plugin: 'yolov8', event: 'vehicle detected', confidence: 0.92, time: new Date(Date.now() - 60000) },
  { id: 3, sensor: 'usb_camera_0', plugin: 'yolov8', event: 'person detected', confidence: 0.76, time: new Date(Date.now() - 120000) },
];

const mockAlertDistribution = [
  { name: 'Person Detection', value: 45, color: '#1e88e5' },
  { name: 'Vehicle Detection', value: 30, color: '#43a047' },
  { name: 'Anomaly Detection', value: 15, color: '#ffca28' },
  { name: 'System Alerts', value: 10, color: '#e53e3e' },
];

const mockPerformanceData = Array.from({ length: 24 }, (_, i) => ({
  time: format(new Date(Date.now() - (23 - i) * 60 * 60 * 1000), 'HH:mm'),
  events: Math.floor(Math.random() * 50) + 10,
  fps: Math.floor(Math.random() * 10) + 25,
}));

function StatsCard({ title, stat, icon, helpText, trend }) {
  return (
    <Stat
      px={{ base: 2, md: 4 }}
      py={'5'}
      shadow={'xl'}
      border={'1px solid'}
      borderColor={useColorModeValue('gray.200', 'gray.500')}
      rounded={'lg'}
      bg={useColorModeValue('white', 'gray.800')}
    >
      <Flex justifyContent={'space-between'}>
        <Box pl={{ base: 2, md: 4 }}>
          <StatLabel fontWeight={'medium'} isTruncated>
            {title}
          </StatLabel>
          <StatNumber fontSize={'2xl'} fontWeight={'medium'}>
            {stat}
          </StatNumber>
          {helpText && (
            <StatHelpText>
              {trend && <StatArrow type={trend} />}
              {helpText}
            </StatHelpText>
          )}
        </Box>
        <Box
          my={'auto'}
          color={useColorModeValue('gray.800', 'gray.200')}
          alignContent={'center'}
        >
          <Icon as={icon} w={8} h={8} />
        </Box>
      </Flex>
    </Stat>
  );
}

export default function Dashboard() {
  const cardBg = useColorModeValue('white', 'gray.800');
  const borderColor = useColorModeValue('gray.200', 'gray.600');

  return (
    <Layout>
      <VStack spacing={8} align="stretch">
        {/* Header */}
        <Box>
          <Heading size="lg" mb={2}>Dashboard Overview</Heading>
          <Text color="gray.600">Monitor your Vargard AI system performance and activity</Text>
        </Box>

        {/* Stats Cards */}
        <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={6}>
          <StatsCard
            title="Active Sensors"
            stat={mockSystemStats.activeSensors}
            icon={FiCamera}
            helpText="All systems operational"
          />
          <StatsCard
            title="Total Alerts"
            stat={mockSystemStats.totalAlerts}
            icon={FiAlertTriangle}
            helpText="Last 24 hours"
            trend="increase"
          />
          <StatsCard
            title="Average FPS"
            stat={`${mockSystemStats.avgFPS}`}
            icon={FiActivity}
            helpText="Across all sensors"
          />
          <StatsCard
            title="System Uptime"
            stat={mockSystemStats.systemUptime}
            icon={FiCpu}
            helpText="Since last restart"
          />
        </SimpleGrid>

        {/* Charts Row */}
        <SimpleGrid columns={{ base: 1, lg: 2 }} spacing={6}>
          {/* Performance Chart */}
          <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
            <Heading size="md" mb={4}>Events & Performance (24h)</Heading>
            <Box height="300px">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart data={mockPerformanceData}>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis dataKey="time" />
                  <YAxis yAxisId="left" />
                  <YAxis yAxisId="right" orientation="right" />
                  <Tooltip />
                  <Line yAxisId="left" type="monotone" dataKey="events" stroke="#1e88e5" strokeWidth={2} />
                  <Line yAxisId="right" type="monotone" dataKey="fps" stroke="#43a047" strokeWidth={2} />
                </LineChart>
              </ResponsiveContainer>
            </Box>
          </Box>

          {/* Alert Distribution */}
          <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
            <Heading size="md" mb={4}>Alert Distribution</Heading>
            <Box height="300px">
              <ResponsiveContainer width="100%" height="100%">
                <PieChart>
                  <Pie
                    data={mockAlertDistribution}
                    cx="50%"
                    cy="50%"
                    outerRadius={80}
                    fill="#8884d8"
                    dataKey="value"
                    label={({ name, percent }) => `${name}: ${(percent * 100).toFixed(0)}%`}
                  >
                    {mockAlertDistribution.map((entry, index) => (
                      <Cell key={`cell-${index}`} fill={entry.color} />
                    ))}
                  </Pie>
                  <Tooltip />
                </PieChart>
              </ResponsiveContainer>
            </Box>
          </Box>
        </SimpleGrid>

        {/* System Resources */}
        <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
          <Heading size="md" mb={4}>System Resources</Heading>
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={6}>
            <Box>
              <Text mb={2} fontWeight="medium">CPU Usage</Text>
              <Progress value={mockSystemStats.cpuUsage} colorScheme="blue" size="lg" />
              <Text fontSize="sm" color="gray.600" mt={1}>{mockSystemStats.cpuUsage}%</Text>
            </Box>
            <Box>
              <Text mb={2} fontWeight="medium">Memory Usage</Text>
              <Progress value={mockSystemStats.memoryUsage} colorScheme="green" size="lg" />
              <Text fontSize="sm" color="gray.600" mt={1}>{mockSystemStats.memoryUsage}%</Text>
            </Box>
            <Box>
              <Text mb={2} fontWeight="medium">GPU Usage</Text>
              <Progress value={mockSystemStats.gpuUsage} colorScheme="orange" size="lg" />
              <Text fontSize="sm" color="gray.600" mt={1}>{mockSystemStats.gpuUsage}%</Text>
            </Box>
          </SimpleGrid>
        </Box>

        {/* Recent Events */}
        <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
          <Heading size="md" mb={4}>Recent Events</Heading>
          <TableContainer>
            <Table variant="simple">
              <Thead>
                <Tr>
                  <Th>Sensor</Th>
                  <Th>Plugin</Th>
                  <Th>Event</Th>
                  <Th>Confidence</Th>
                  <Th>Time</Th>
                </Tr>
              </Thead>
              <Tbody>
                {mockRecentEvents.map((event) => (
                  <Tr key={event.id}>
                    <Td>
                      <Badge colorScheme="blue" variant="subtle">
                        {event.sensor}
                      </Badge>
                    </Td>
                    <Td>{event.plugin}</Td>
                    <Td>{event.event}</Td>
                    <Td>
                      <Badge 
                        colorScheme={event.confidence > 0.8 ? 'green' : 'yellow'}
                        variant="subtle"
                      >
                        {(event.confidence * 100).toFixed(0)}%
                      </Badge>
                    </Td>
                    <Td>{format(event.time, 'HH:mm:ss')}</Td>
                  </Tr>
                ))}
              </Tbody>
            </Table>
          </TableContainer>
        </Box>
      </VStack>
    </Layout>
  );
}
