import { ChakraProvider, extendTheme } from '@chakra-ui/react';
import { QueryClient, QueryClientProvider } from 'react-query';
import { Toaster } from 'react-hot-toast';
import { useState } from 'react';

// Custom theme for Vargard
const theme = extendTheme({
  colors: {
    brand: {
      50: '#e3f2fd',
      100: '#bbdefb',
      200: '#90caf9',
      300: '#64b5f6',
      400: '#42a5f5',
      500: '#1e88e5', // Primary blue
      600: '#1976d2',
      700: '#1565c0',
      800: '#0d47a1',
      900: '#0a2e5c',
    },
    success: {
      500: '#43a047', // Green
    },
    warning: {
      500: '#ffca28', // Amber
    },
  },
  fonts: {
    heading: 'Inter, sans-serif',
    body: 'Inter, sans-serif',
  },
  config: {
    initialColorMode: 'light',
    useSystemColorMode: false,
  },
});

export default function App({ Component, pageProps }) {
  const [queryClient] = useState(() => new QueryClient({
    defaultOptions: {
      queries: {
        refetchOnWindowFocus: false,
        retry: 1,
        staleTime: 30000, // 30 seconds
      },
    },
  }));

  return (
    <QueryClientProvider client={queryClient}>
      <ChakraProvider theme={theme}>
        <Component {...pageProps} />
        <Toaster 
          position="top-right"
          toastOptions={{
            duration: 4000,
            style: {
              background: '#363636',
              color: '#fff',
            },
          }}
        />
      </ChakraProvider>
    </QueryClientProvider>
  );
}