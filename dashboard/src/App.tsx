import React from 'react';
import logo from './logo.svg';
import './App.css';
import Button from '@mui/material/Button';
import { createTheme, ThemeProvider, styled} from '@mui/material/styles';
import useMediaQuery from '@mui/material/useMediaQuery';
import Paper from '@mui/material/Paper';
import CssBaseline from '@mui/material/CssBaseline';
import {dark, light} from 'ayu'
import AppDrawer from './AppDrawer'

function App() {
  // Detects user's system theme preference
  const prefersDarkMode = useMediaQuery('(prefers-color-scheme: dark)');

  // React hook to create a theme based on user preference
  const theme = React.useMemo(() =>
    createTheme({
      palette: {
        mode: prefersDarkMode ? 'dark' : 'light',
        // Setting primary and background colors using ayu
        primary: {
          main: prefersDarkMode ? dark.ui.bg.hex() : light.ui.bg.hex(),
        },
        background: {
          default: prefersDarkMode ? dark.ui.bg.hex() : light.ui.bg.hex(),
          paper: prefersDarkMode ? dark.ui.panel.bg.hex() :light.ui.panel.bg.hex(),
        },
        text: {
          primary: prefersDarkMode ? dark.editor.fg.hex() : light.editor.fg.hex(),
          secondary: prefersDarkMode ? dark.syntax.comment.hex() : light.syntax.comment.hex(),
        }
      },
    }),
    [prefersDarkMode]
  );

  const StyledPaper = styled(Paper)(({ theme }) => ({
    margin: 'auto',
    marginTop: '5vh',
    width: '80vw',
    height: '80vh',
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: theme.palette.background.paper,
    boxShadow: '0px 4px 20px rgba(0, 0, 0, 0.5)',
    borderRadius: theme.shape.borderRadius,
    overflow: 'hidden',
    transform: 'translateZ(0)',
  }));

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline /> {/* Applies baseline CSS for consistent styling */}
      <StyledPaper elevation={24}>
        <AppDrawer />
      </StyledPaper>
    </ThemeProvider>
  );
}

export default App;
