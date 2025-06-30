import {
  Box,
  Flex,
  Text,
  VStack,
  HStack,
  Icon,
  Link,
  useColorModeValue,
  Divider,
  Badge,
  Avatar,
  Menu,
  MenuButton,
  MenuList,
  MenuItem,
  IconButton,
  useDisclosure,
  Drawer,
  DrawerOverlay,
  DrawerContent,
  DrawerCloseButton,
  DrawerHeader,
  DrawerBody,
} from '@chakra-ui/react';
import {
  FiHome,
  FiMonitor,
  FiActivity,
  FiAlertTriangle,
  FiSettings,
  FiPlay,
  FiCpu,
  FiMenu,
  FiUser,
  FiLogOut,
} from 'react-icons/fi';
import { useRouter } from 'next/router';
import NextLink from 'next/link';

const navItems = [
  { name: 'Dashboard', href: '/', icon: FiHome },
  { name: 'Live View', href: '/live', icon: FiMonitor },
  { name: 'Events', href: '/events', icon: FiActivity },
  { name: 'Alerts', href: '/alerts', icon: FiAlertTriangle },
  { name: 'Telemetry', href: '/telemetry', icon: FiCpu },
  { name: 'Rules', href: '/rules', icon: FiPlay },
  { name: 'Plugins', href: '/plugins', icon: FiSettings },
];

const SidebarContent = ({ onClose, ...rest }) => {
  const router = useRouter();
  
  return (
    <Box
      bg={useColorModeValue('white', 'gray.900')}
      borderRight="1px"
      borderRightColor={useColorModeValue('gray.200', 'gray.700')}
      w={{ base: 'full', md: 60 }}
      pos="fixed"
      h="full"
      {...rest}
    >
      <Flex h="20" alignItems="center" mx="8" justifyContent="space-between">
        <Text fontSize="2xl" fontFamily="monospace" fontWeight="bold" color="brand.500">
          Vargard
        </Text>
        <Badge colorScheme="green" variant="subtle">
          v0.1.0
        </Badge>
      </Flex>
      
      <Divider />
      
      <VStack spacing={1} align="stretch" mt={4}>
        {navItems.map((item) => (
          <NextLink key={item.name} href={item.href} passHref>
            <Link
              style={{ textDecoration: 'none' }}
              _focus={{ boxShadow: 'none' }}
            >
              <Flex
                align="center"
                p="4"
                mx="4"
                borderRadius="lg"
                role="group"
                cursor="pointer"
                bg={router.pathname === item.href ? 'brand.50' : 'transparent'}
                color={router.pathname === item.href ? 'brand.600' : 'gray.600'}
                _hover={{
                  bg: 'brand.50',
                  color: 'brand.600',
                }}
              >
                <Icon
                  mr="4"
                  fontSize="16"
                  as={item.icon}
                />
                {item.name}
              </Flex>
            </Link>
          </NextLink>
        ))}
      </VStack>
    </Box>
  );
};

const MobileNav = ({ onOpen, ...rest }) => {
  return (
    <Flex
      ml={{ base: 0, md: 60 }}
      px={{ base: 4, md: 4 }}
      height="20"
      alignItems="center"
      bg={useColorModeValue('white', 'gray.900')}
      borderBottomWidth="1px"
      borderBottomColor={useColorModeValue('gray.200', 'gray.700')}
      justifyContent={{ base: 'space-between', md: 'flex-end' }}
      {...rest}
    >
      <IconButton
        display={{ base: 'flex', md: 'none' }}
        onClick={onOpen}
        variant="outline"
        aria-label="open menu"
        icon={<FiMenu />}
      />

      <Text
        display={{ base: 'flex', md: 'none' }}
        fontSize="2xl"
        fontFamily="monospace"
        fontWeight="bold"
        color="brand.500"
      >
        Vargard
      </Text>

      <HStack spacing={{ base: '0', md: '6' }}>
        <Menu>
          <MenuButton
            py={2}
            transition="all 0.3s"
            _focus={{ boxShadow: 'none' }}
          >
            <HStack>
              <Avatar
                size={'sm'}
                src={'https://images.unsplash.com/photo-1619946794135-5bc917a27793?ixlib=rb-0.3.5&q=80&fm=jpg&crop=faces&fit=crop&h=200&w=200&s=b616b2c5b373a80ffc9636ba24f7a4a9'}
              />
              <VStack
                display={{ base: 'none', md: 'flex' }}
                alignItems="flex-start"
                spacing="1px"
                ml="2"
              >
                <Text fontSize="sm">Operator</Text>
                <Text fontSize="xs" color="gray.600">
                  Admin
                </Text>
              </VStack>
            </HStack>
          </MenuButton>
          <MenuList
            bg={useColorModeValue('white', 'gray.900')}
            borderColor={useColorModeValue('gray.200', 'gray.700')}
          >
            <MenuItem icon={<FiUser />}>Profile</MenuItem>
            <MenuItem icon={<FiSettings />}>Settings</MenuItem>
            <Divider />
            <MenuItem icon={<FiLogOut />}>Sign out</MenuItem>
          </MenuList>
        </Menu>
      </HStack>
    </Flex>
  );
};

export default function Layout({ children }) {
  const { isOpen, onOpen, onClose } = useDisclosure();
  
  return (
    <Box minH="100vh" bg={useColorModeValue('gray.100', 'gray.900')}>
      <SidebarContent
        onClose={onClose}
        display={{ base: 'none', md: 'block' }}
      />
      <Drawer
        autoFocus={false}
        isOpen={isOpen}
        placement="left"
        onClose={onClose}
        returnFocusOnClose={false}
        onOverlayClick={onClose}
        size="full"
      >
        <DrawerOverlay />
        <DrawerContent>
          <SidebarContent onClose={onClose} />
        </DrawerContent>
      </Drawer>
      
      <MobileNav onOpen={onOpen} />
      
      <Box ml={{ base: 0, md: 60 }} p="4">
        {children}
      </Box>
    </Box>
  );
}