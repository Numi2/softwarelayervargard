import { useState, useEffect, useRef } from 'react';
import {
  Box,
  SimpleGrid,
  VStack,
  HStack,
  Heading,
  Text,
  Badge,
  Button,
  IconButton,
  Select,
  Switch,
  FormControl,
  FormLabel,
  useColorModeValue,
  Modal,
  ModalOverlay,
  ModalContent,
  ModalHeader,
  ModalFooter,
  ModalBody,
  ModalCloseButton,
  useDisclosure,
  Tooltip,
  Slider,
  SliderTrack,
  SliderFilledTrack,
  SliderThumb,
  Alert,
  AlertIcon,
  Spinner,
} from '@chakra-ui/react';
import {
  FiPlay,
  FiPause,
  FiMaximize,
  FiMinimize,
  FiRefreshCw,
  FiSettings,
  FiEye,
  FiEyeOff,
  FiGrid,
  FiMonitor,
} from 'react-icons/fi';
import { useQuery } from 'react-query';
import axios from 'axios';
import Layout from '../components/Layout';

// Mock camera data - replace with real API calls
const mockCameras = [
  {
    id: 'usb_camera_0',
    name: 'USB Camera 0',
    status: 'active',
    fps: 30,
    resolution: '1920x1080',
    detections: [
      { class: 'person', confidence: 0.87, bbox: [100, 50, 200, 300] },
      { class: 'vehicle', confidence: 0.92, bbox: [300, 100, 500, 250] },
    ],
    lastFrame: Date.now(),
  },
  {
    id: 'csi_camera',
    name: 'CSI Camera',
    status: 'active',
    fps: 30,
    resolution: '1920x1080',
    detections: [
      { class: 'person', confidence: 0.76, bbox: [150, 80, 250, 350] },
    ],
    lastFrame: Date.now(),
  },
  {
    id: 'ip_camera_1',
    name: 'IP Camera 1',
    status: 'disconnected',
    fps: 0,
    resolution: '1280x720',
    detections: [],
    lastFrame: Date.now() - 30000,
  },
  {
    id: 'ip_camera_2',
    name: 'IP Camera 2',
    status: 'active',
    fps: 25,
    resolution: '1280x720',
    detections: [],
    lastFrame: Date.now(),
  },
];

const CameraFeed = ({ camera, showDetections, onFullscreen }) => {
  const [isPlaying, setIsPlaying] = useState(true);
  const videoRef = useRef(null);
  const canvasRef = useRef(null);
  
  const cardBg = useColorModeValue('white', 'gray.800');
  const borderColor = useColorModeValue('gray.200', 'gray.600');

  // Simulate video feed with canvas
  useEffect(() => {
    if (!isPlaying || camera.status !== 'active') return;

    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    let animationId;

    const drawFrame = () => {
      // Clear canvas
      ctx.fillStyle = '#1a202c';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      // Draw simulated video content
      ctx.fillStyle = '#2d3748';
      ctx.fillRect(20, 20, canvas.width - 40, canvas.height - 40);

      // Draw camera info
      ctx.fillStyle = '#ffffff';
      ctx.font = '16px Arial';
      ctx.fillText(`${camera.name} - ${camera.fps} FPS`, 30, 50);
      ctx.fillText(`${camera.resolution}`, 30, 70);

      // Draw detection overlays if enabled
      if (showDetections && camera.detections.length > 0) {
        camera.detections.forEach((detection, idx) => {
          const [x, y, w, h] = detection.bbox;
          
          // Scale bbox to canvas size
          const scaleX = canvas.width / 1920;
          const scaleY = canvas.height / 1080;
          const scaledX = x * scaleX;
          const scaledY = y * scaleY;
          const scaledW = (w - x) * scaleX;
          const scaledH = (h - y) * scaleY;

          // Draw bounding box
          ctx.strokeStyle = detection.class === 'person' ? '#48bb78' : '#ed8936';
          ctx.lineWidth = 2;
          ctx.strokeRect(scaledX, scaledY, scaledW, scaledH);

          // Draw label
          ctx.fillStyle = detection.class === 'person' ? '#48bb78' : '#ed8936';
          ctx.fillRect(scaledX, scaledY - 25, scaledW, 25);
          ctx.fillStyle = '#ffffff';
          ctx.font = '12px Arial';
          ctx.fillText(
            `${detection.class} (${(detection.confidence * 100).toFixed(0)}%)`,
            scaledX + 5,
            scaledY - 8
          );
        });
      }

      // Draw timestamp
      ctx.fillStyle = '#ffffff';
      ctx.font = '12px Arial';
      ctx.fillText(new Date().toLocaleTimeString(), canvas.width - 100, canvas.height - 20);

      animationId = requestAnimationFrame(drawFrame);
    };

    drawFrame();

    return () => {
      if (animationId) {
        cancelAnimationFrame(animationId);
      }
    };
  }, [camera, isPlaying, showDetections]);

  const getStatusColor = (status) => {
    switch (status) {
      case 'active': return 'green';
      case 'disconnected': return 'red';
      case 'reconnecting': return 'yellow';
      default: return 'gray';
    }
  };

  return (
    <Box
      bg={cardBg}
      rounded="lg"
      border="1px"
      borderColor={borderColor}
      overflow="hidden"
      position="relative"
    >
      {/* Camera Header */}
      <HStack justify="space-between" p={3} bg={useColorModeValue('gray.50', 'gray.700')}>
        <VStack align="start" spacing={0}>
          <Text fontWeight="bold" fontSize="sm">{camera.name}</Text>
          <HStack spacing={2}>
            <Badge colorScheme={getStatusColor(camera.status)} size="sm">
              {camera.status}
            </Badge>
            <Text fontSize="xs" color="gray.500">{camera.resolution}</Text>
            {camera.status === 'active' && (
              <Text fontSize="xs" color="gray.500">{camera.fps} FPS</Text>
            )}
          </HStack>
        </VStack>
        
        <HStack spacing={1}>
          <Tooltip label={isPlaying ? 'Pause' : 'Play'}>
            <IconButton
              size="sm"
              icon={isPlaying ? <FiPause /> : <FiPlay />}
              onClick={() => setIsPlaying(!isPlaying)}
              variant="ghost"
              isDisabled={camera.status !== 'active'}
            />
          </Tooltip>
          
          <Tooltip label="Fullscreen">
            <IconButton
              size="sm"
              icon={<FiMaximize />}
              onClick={() => onFullscreen(camera)}
              variant="ghost"
            />
          </Tooltip>
        </HStack>
      </HStack>

      {/* Video Feed */}
      <Box position="relative" bg="black" minH="200px">
        {camera.status === 'active' ? (
          <canvas
            ref={canvasRef}
            width={400}
            height={300}
            style={{ width: '100%', height: 'auto', display: 'block' }}
          />
        ) : (
          <VStack justify="center" minH="200px" color="gray.500">
            {camera.status === 'disconnected' ? (
              <>
                <FiEyeOff size={40} />
                <Text>Camera Disconnected</Text>
                <Button size="sm" leftIcon={<FiRefreshCw />} variant="outline">
                  Reconnect
                </Button>
              </>
            ) : (
              <>
                <Spinner size="lg" />
                <Text>Connecting...</Text>
              </>
            )}
          </VStack>
        )}

        {/* Detection Count Overlay */}
        {camera.status === 'active' && showDetections && camera.detections.length > 0 && (
          <Badge
            position="absolute"
            top={2}
            right={2}
            colorScheme="blue"
            variant="solid"
          >
            {camera.detections.length} detection{camera.detections.length !== 1 ? 's' : ''}
          </Badge>
        )}
      </Box>
    </Box>
  );
};

const FullscreenModal = ({ camera, isOpen, onClose, showDetections }) => {
  const canvasRef = useRef(null);

  useEffect(() => {
    if (!isOpen || !camera || camera.status !== 'active') return;

    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    let animationId;

    const drawFrame = () => {
      // Clear canvas
      ctx.fillStyle = '#1a202c';
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      // Draw simulated video content
      ctx.fillStyle = '#2d3748';
      ctx.fillRect(40, 40, canvas.width - 80, canvas.height - 80);

      // Draw camera info
      ctx.fillStyle = '#ffffff';
      ctx.font = '24px Arial';
      ctx.fillText(`${camera.name} - ${camera.fps} FPS`, 60, 100);
      ctx.fillText(`${camera.resolution}`, 60, 130);

      // Draw detection overlays if enabled
      if (showDetections && camera.detections.length > 0) {
        camera.detections.forEach((detection) => {
          const [x, y, w, h] = detection.bbox;
          
          // Scale bbox to canvas size
          const scaleX = canvas.width / 1920;
          const scaleY = canvas.height / 1080;
          const scaledX = x * scaleX;
          const scaledY = y * scaleY;
          const scaledW = (w - x) * scaleX;
          const scaledH = (h - y) * scaleY;

          // Draw bounding box
          ctx.strokeStyle = detection.class === 'person' ? '#48bb78' : '#ed8936';
          ctx.lineWidth = 3;
          ctx.strokeRect(scaledX, scaledY, scaledW, scaledH);

          // Draw label
          ctx.fillStyle = detection.class === 'person' ? '#48bb78' : '#ed8936';
          ctx.fillRect(scaledX, scaledY - 35, scaledW, 35);
          ctx.fillStyle = '#ffffff';
          ctx.font = '18px Arial';
          ctx.fillText(
            `${detection.class} (${(detection.confidence * 100).toFixed(0)}%)`,
            scaledX + 10,
            scaledY - 10
          );
        });
      }

      // Draw timestamp
      ctx.fillStyle = '#ffffff';
      ctx.font = '18px Arial';
      ctx.fillText(new Date().toLocaleTimeString(), canvas.width - 150, canvas.height - 30);

      animationId = requestAnimationFrame(drawFrame);
    };

    drawFrame();

    return () => {
      if (animationId) {
        cancelAnimationFrame(animationId);
      }
    };
  }, [camera, isOpen, showDetections]);

  return (
    <Modal isOpen={isOpen} onClose={onClose} size="full">
      <ModalOverlay bg="blackAlpha.900" />
      <ModalContent bg="black" m={0}>
        <ModalHeader color="white">
          <HStack justify="space-between">
            <Text>{camera?.name} - Live View</Text>
            <IconButton
              icon={<FiMinimize />}
              onClick={onClose}
              variant="ghost"
              color="white"
            />
          </HStack>
        </ModalHeader>
        <ModalBody p={0}>
          {camera && camera.status === 'active' ? (
            <canvas
              ref={canvasRef}
              width={1920}
              height={1080}
              style={{ width: '100%', height: 'calc(100vh - 80px)', objectFit: 'contain' }}
            />
          ) : (
            <VStack justify="center" h="calc(100vh - 80px)" color="white">
              <FiEyeOff size={60} />
              <Text fontSize="xl">Camera not available</Text>
            </VStack>
          )}
        </ModalBody>
      </ModalContent>
    </Modal>
  );
};

export default function LiveView() {
  const [gridSize, setGridSize] = useState('2x2');
  const [showDetections, setShowDetections] = useState(true);
  const [selectedCamera, setSelectedCamera] = useState(null);
  const [cameras, setCameras] = useState(mockCameras);
  
  const { isOpen, onOpen, onClose } = useDisclosure();
  const cardBg = useColorModeValue('white', 'gray.800');
  const borderColor = useColorModeValue('gray.200', 'gray.600');

  // Simulate real-time updates
  useEffect(() => {
    const interval = setInterval(() => {
      setCameras(prevCameras => 
        prevCameras.map(camera => ({
          ...camera,
          lastFrame: camera.status === 'active' ? Date.now() : camera.lastFrame,
          detections: camera.status === 'active' ? 
            // Randomly update detections
            Math.random() > 0.7 ? 
              camera.detections.map(det => ({
                ...det,
                confidence: Math.max(0.5, Math.min(0.99, det.confidence + (Math.random() - 0.5) * 0.1))
              })) : camera.detections
            : []
        }))
      );
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  const handleFullscreen = (camera) => {
    setSelectedCamera(camera);
    onOpen();
  };

  const getGridColumns = () => {
    switch (gridSize) {
      case '1x1': return 1;
      case '2x2': return 2;
      case '3x3': return 3;
      case '4x4': return 4;
      default: return 2;
    }
  };

  const activeCameras = cameras.filter(cam => cam.status === 'active');
  const totalDetections = activeCameras.reduce((sum, cam) => sum + cam.detections.length, 0);

  return (
    <Layout>
      <VStack spacing={6} align="stretch">
        {/* Header */}
        <HStack justify="space-between" align="center">
          <Box>
            <Heading size="lg" mb={2}>Live Camera Feeds</Heading>
            <HStack>
              <Text color="gray.600">Real-time monitoring with AI detection</Text>
              <Badge colorScheme="green" variant="subtle">
                {activeCameras.length} Active
              </Badge>
              {totalDetections > 0 && (
                <Badge colorScheme="blue" variant="subtle">
                  {totalDetections} Detection{totalDetections !== 1 ? 's' : ''}
                </Badge>
              )}
            </HStack>
          </Box>

          <HStack spacing={4}>
            <FormControl display="flex" alignItems="center">
              <FormLabel htmlFor="detections-toggle" mb="0" fontSize="sm">
                Show Detections
              </FormLabel>
              <Switch
                id="detections-toggle"
                isChecked={showDetections}
                onChange={(e) => setShowDetections(e.target.checked)}
                colorScheme="blue"
              />
            </FormControl>

            <Box>
              <Text fontSize="sm" mb={1}>Grid Layout</Text>
              <Select value={gridSize} onChange={(e) => setGridSize(e.target.value)} size="sm" w="100px">
                <option value="1x1">1×1</option>
                <option value="2x2">2×2</option>
                <option value="3x3">3×3</option>
                <option value="4x4">4×4</option>
              </Select>
            </Box>
          </HStack>
        </HStack>

        {/* System Status */}
        {cameras.some(cam => cam.status === 'disconnected') && (
          <Alert status="warning" rounded="lg">
            <AlertIcon />
            <Box>
              <Text fontWeight="bold">Camera Issues Detected</Text>
              <Text fontSize="sm">
                {cameras.filter(cam => cam.status === 'disconnected').length} camera(s) are disconnected
              </Text>
            </Box>
          </Alert>
        )}

        {/* Camera Grid */}
        <SimpleGrid columns={getGridColumns()} spacing={6}>
          {cameras.slice(0, parseInt(gridSize.split('x')[0]) * parseInt(gridSize.split('x')[1])).map((camera) => (
            <CameraFeed
              key={camera.id}
              camera={camera}
              showDetections={showDetections}
              onFullscreen={handleFullscreen}
            />
          ))}
        </SimpleGrid>

        {/* Detection Summary */}
        {showDetections && totalDetections > 0 && (
          <Box bg={cardBg} p={6} rounded="lg" border="1px" borderColor={borderColor}>
            <Heading size="md" mb={4}>Current Detections</Heading>
            <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={4}>
              {activeCameras.map((camera) => (
                camera.detections.length > 0 && (
                  <Box key={camera.id} p={4} border="1px" borderColor={borderColor} rounded="md">
                    <Text fontWeight="bold" mb={2}>{camera.name}</Text>
                    <VStack align="start" spacing={1}>
                      {camera.detections.map((detection, idx) => (
                        <HStack key={idx} justify="space-between" w="full">
                          <Text fontSize="sm">{detection.class}</Text>
                          <Badge 
                            colorScheme={detection.confidence > 0.8 ? 'green' : 'yellow'}
                            size="sm"
                          >
                            {(detection.confidence * 100).toFixed(0)}%
                          </Badge>
                        </HStack>
                      ))}
                    </VStack>
                  </Box>
                )
              ))}
            </SimpleGrid>
          </Box>
        )}

        {/* Fullscreen Modal */}
        <FullscreenModal
          camera={selectedCamera}
          isOpen={isOpen}
          onClose={onClose}
          showDetections={showDetections}
        />
      </VStack>
    </Layout>
  );
}
