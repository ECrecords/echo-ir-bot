import * as React from 'react';
import Box from '@mui/material/Box';
import Drawer from '@mui/material/Drawer';
import CssBaseline from '@mui/material/CssBaseline';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import List from '@mui/material/List';
import Typography from '@mui/material/Typography';
import Divider from '@mui/material/Divider';
import ListItem from '@mui/material/ListItem';
import ListItemButton from '@mui/material/ListItemButton';
import ListItemIcon from '@mui/material/ListItemIcon';
import ListItemText from '@mui/material/ListItemText';
import SettingsApplicationsIcon from '@mui/icons-material/SettingsApplications';
import RadarIcon from '@mui/icons-material/Radar';
import ShowChartIcon from '@mui/icons-material/ShowChart';

import RadarDisplay from './RadarDisplay';
import LineChartDisplay from './LineChartDisplay';

// Configuration.js
function Configuration() {
    return <p>This is the Configuration component.</p>;
}
  

const drawerWidth = 240;

export default function AppDrawer() {
  const [selectedComponent, setSelectedComponent] = React.useState('Configuration');

  const handleListItemClick = (component: any) => {
    setSelectedComponent(component);
  };

  return (
    <Box sx={{ display: 'flex' }}>
      <CssBaseline />
      <AppBar
        position="fixed"
        sx={{ width: `calc(100% - ${drawerWidth}px)`, ml: `${drawerWidth}px` }}
      >
        <Toolbar>
          <Typography variant="h6" noWrap component="div">
            Radar Navagation
          </Typography>
        </Toolbar>
      </AppBar>
      <Drawer
        sx={{
          width: drawerWidth,
          flexShrink: 0,
          '& .MuiDrawer-paper': {
            width: drawerWidth,
            boxSizing: 'border-box',
          },
        }}
        variant="permanent"
        anchor="left"
      >
        <Toolbar />
        <Divider />
        <List>
          {['Configuration', 'Data', 'Radar'].map((text) => (
            <ListItem key={text} disablePadding>
              <ListItemButton onClick={() => handleListItemClick(text)}>
                <ListItemIcon>
                  {text === 'Configuration' && <SettingsApplicationsIcon />}
                  {text === 'Data' && <ShowChartIcon />}
                  {text === 'Radar' && <RadarIcon />}
                </ListItemIcon>
                <ListItemText primary={text} />
              </ListItemButton>
            </ListItem>
          ))}
        </List>
      </Drawer>
      <Box
        component="main"
        sx={{ flexGrow: 1, bgcolor: 'background.default', p: 3 }}
      >
        <Toolbar />
        {selectedComponent === 'Configuration' && <Configuration />}
        {selectedComponent === 'Radar' && <RadarDisplay angle={90} distance={100} />}
        {selectedComponent === 'Data' && <LineChartDisplay />}

      </Box>
    </Box>
  );
}