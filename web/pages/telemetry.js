import { useEffect, useState } from 'react';
import {
  Box,
  SimpleGrid,
  Heading,
  Text,
  VStack,
  HStack,
  Progress,
  Badge,
  useColorModeValue,
  Stat,
  StatLabel,
  StatNumber,
  StatHelpText,
  Select,
  Button,
  ButtonGroup,
} from '@chakra-ui/react';
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';
import { format } from 'date-fns';
import Layout from '../components/Layout';

export default function Telemetry() {
  const [data, setData] = useState([]);
  const [timeRange, setTimeRange] = useState('1h');
  const [isConnected, setIsConnected] = useState(false);
  
  const cardBg = useColorModeValue('white', 'gray.800');
  const borderColor = useColorModeValue('gray.200', 'gray.600');

  useEffect(() => {
    const evt = new EventSource('http://localhost:5000/telemetry/stream');
    
    evt.onopen = () => {
      setIsConnected(true);
    };
    
    evt.onmessage = (e) => {
      try {
        const newData = JSON.parse(e.data);
        setData((prevData) => {
          const updated = [...prevData.slice(-99), {
            ...newData,
            timestamp: new Date(newData.timestamp * 1000),
          }];
          return updated;
        });
      } catch (error) {
        console.error('Error parsing telemetry data:', error);
      }
    };
    
    evt.onerror = () => {
      setIsConnected(false);
    };
    
    return () => {
      evt.close();
      setIsConnected(false);
    };
  }, []);

  const latestData = data[data.length - 1] || {};
  
  // Filter data based on time range
  const getFilteredData = () => {
    const now = new Date();
    const ranges = {
      '1h': 60 * 60 * 1000,
      '6h': 6 * 60 * 60 * 1000,
      '24h': 24 * 60 * 60 * 1000,
    };
    
    const cutoff = new Date(now.getTime() - ranges[timeRange]);
    return data.filter(d => d.timestamp >= cutoff);
  };

  const filteredData = getFilteredData();

  const formatDataForChart = (key) => {
    return filteredData.map(d => ({
      time: format(d.timestamp, 'HH:mm'),
      value: d[key] || 0,
    }));
  };

  return (
    <Layout>
      <VStack spacing={8} align="stretch">
        {/* Header */}
        <HStack justify="space-between" align="center">
          <Box>
            <Heading size="lg" mb={2}>System Telemetry</Heading>
            <HStack>
              <Text color="gray.600">Real-time system monitoring</Text>
              <Badge colorScheme={isConnected ? 'green' : 'red'} variant="subtle">
                {isConnected ? 'Connected' : 'Disconnected'}
              </Badge>
            </HStack>
          </Box>
          
          <HStack>
            <Text fontSize="sm">Time Range:</Text>
            <Select value={timeRange} onChange={(e) => setTimeRange(e.target.value)} size="sm" w="100px">
              <option value="1h">1 Hour</option>
              <option value="6h">6 Hours</option>
              <option value="24h">24 Hours</option>
            </Select>
          </HStack>
        </HStack>

        {/* Current Stats */}
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
            <StatLabel>CPU Usage</StatLabel>
            <StatNumber>{latestData.cpu_percent?.toFixed(1) || 0}%</StatNumber>
            <Progress 
              value={latestData.cpu_percent || 0} 
              colorScheme="blue" 
              size="sm" 
              mt={2}
            />
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
            <StatLabel>Memory Usage</StatLabel>
            <StatNumber>{latestData.memory_percent?.toFixed(1) || 0}%</StatNumber>
            <Progress 
              value={latestData.memory_percent || 0} 
              colorScheme="green" 
              size="sm" 
              mt={2}
            />
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
            <StatLabel>GPU Usage</StatLabel>
            <StatNumber>{latestData.gpu?.utilization?.toFixed(1) || 0}%</StatNumber>
            <Progress 
              value={latestData.gpu?.utilization || 0} 
              colorScheme="orange" 
              size="sm" 
              mt={2}
            />
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
            <StatLabel>GPU Temperature</StatLabel>
            <StatNumber>{latestData.gpu?.temperature?.toFixed(0) || 0}°C</StatNumber>
            <StatHelpText>
              {latestData.gpu?.temperature > 80 ? 'High' : latestData.gpu?.temperature > 60 ? 'Normal' : 'Cool'}
            </StatHelpText>
          </Stat>
        </SimpleGrid>

        {/* Charts */}
        <SimpleGrid columns={{ base: 1, lg: 2 }} spacing={6}>
          {/* CPU & Memory Chart */}
          <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
            <Heading size="md" mb={4}>CPU & Memory Usage</Heading>
            <Box height="300px">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis dataKey="time" />
                  <YAxis domain={[0, 100]} />
                  <Tooltip formatter={(value) => [`${value}%`, '']} />
                  <Line 
                    data={formatDataForChart('cpu_percent')}
                    type="monotone" 
                    dataKey="value" 
                    stroke="#1e88e5" 
                    strokeWidth={2}
                    name="CPU"
                  />
                  <Line 
                    data={formatDataForChart('memory_percent')}
                    type="monotone" 
                    dataKey="value" 
                    stroke="#43a047" 
                    strokeWidth={2}
                    name="Memory"
                  />
                </LineChart>
              </ResponsiveContainer>
            </Box>
          </Box>

          {/* Network Chart */}
          <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
            <Heading size="md" mb={4}>Network Activity</Heading>
            <Box height="300px">
              <ResponsiveContainer width="100%" height="100%">
                <LineChart>
                  <CartesianGrid strokeDasharray="3 3" />
                  <XAxis dataKey="time" />
                  <YAxis />
                  <Tooltip formatter={(value) => [`${(value / 1024).toFixed(2)} KB/s`, '']} />
                  <Line 
                    data={formatDataForChart('network_bytes_sent')}
                    type="monotone" 
                    dataKey="value" 
                    stroke="#ff6b6b" 
                    strokeWidth={2}
                    name="Sent"
                  />
                  <Line 
                    data={formatDataForChart('network_bytes_recv')}
                    type="monotone" 
                    dataKey="value" 
                    stroke="#4ecdc4" 
                    strokeWidth={2}
                    name="Received"
                  />
                </LineChart>
              </ResponsiveContainer>
            </Box>
          </Box>
        </SimpleGrid>

        {/* Temperature Data */}
        {latestData.temperatures && Object.keys(latestData.temperatures).length > 0 && (
          <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
            <Heading size="md" mb={4}>System Temperatures</Heading>
            <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={4}>
              {Object.entries(latestData.temperatures).map(([sensor, temps]) => (
                <Box key={sensor} p={4} border="1px" borderColor={borderColor} rounded="md">
                  <Text fontWeight="medium" mb={2}>{sensor}</Text>
                  {Array.isArray(temps) ? temps.map((temp, idx) => (
                    <HStack key={idx} justify="space-between">
                      <Text fontSize="sm">Sensor {idx + 1}</Text>
                      <Badge colorScheme={temp > 80 ? 'red' : temp > 60 ? 'yellow' : 'green'}>
                        {temp.toFixed(1)}°C
                      </Badge>
                    </HStack>
                  )) : (
                    <HStack justify="space-between">
                      <Text fontSize="sm">Temperature</Text>
                      <Badge colorScheme={temps > 80 ? 'red' : temps > 60 ? 'yellow' : 'green'}>
                        {temps.toFixed(1)}°C
                      </Badge>
                    </HStack>
                  )}
                </Box>
              ))}
            </SimpleGrid>
          </Box>
        )}
      </VStack>
    </Layout>
  );
}
