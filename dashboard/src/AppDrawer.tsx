import * as React from 'react';
import Box from '@mui/material/Box';
import Drawer from '@mui/material/Drawer';
import AppBar from '@mui/material/AppBar';
import CssBaseline from '@mui/material/CssBaseline';
import Toolbar from '@mui/material/Toolbar';
import List from '@mui/material/List';
import Stack from '@mui/material/Stack';
import Typography from '@mui/material/Typography';
import Divider from '@mui/material/Divider';
import ListItem from '@mui/material/ListItem';
import ListItemButton from '@mui/material/ListItemButton';
import ListItemIcon from '@mui/material/ListItemIcon';
import ListItemText from '@mui/material/ListItemText';

import SettingIcon from '@mui/icons-material/Settings';
import DataIcon from '@mui/icons-material/ShowChart';
import RadarIcon from '@mui/icons-material/Radar';
import { Button } from '@mui/material';


import SimpleLineChart from './DisplayData';

const drawerWidth = 240;

interface Data {
  distance: Array<number>;
  angle: Array<number>;
}


function AppDrawer() {

  // state for slected item
  const [selectedItem, setSelectedItem] = React.useState('Configurations');
  const [startDisabled, setStartDisabled] = React.useState(true);
  const [stopDisabled, setStopDisabled] = React.useState(false);
  const [samples, setSamples] = React.useState<Data>({
    distance: [],
    angle: []
  });

  React.useEffect( () => {
    const fetchData = async () => {
      try {
        const response = await fetch('http://localhost:3000/data')   
        const updateSamples = await response.json();
        setSamples(updateSamples);
      } catch (error) {
        console.error(error);
      }
    };

    fetchData();

    const interval = setInterval(fetchData, 500);

    return () => clearInterval(interval);
  }, []);



  const startButtonClickHandler = () => {
    console.log('start button clicked');
    setStartDisabled(true);  // Disable the start button
    setStopDisabled(false);  // Enable the stop button
  }

  const stopButtonClickHandler = () => {
    console.log('stop button clicked');
    setStopDisabled(true);  // Enable the start button
    setStartDisabled(false)
  }

  return (
    <Box sx={{ display: 'flex' }}>
      <CssBaseline />
      <AppBar position="fixed" sx={{ zIndex: (theme) => theme.zIndex.drawer + 1 }}>
        <Toolbar>
          <Typography variant="h6" noWrap component="div">
            Clipped drawer
          </Typography>
        </Toolbar>
      </AppBar>
      <Drawer
        variant="permanent"
        sx={{
          width: drawerWidth,
          flexShrink: 0,
          [`& .MuiDrawer-paper`]: { width: drawerWidth, boxSizing: 'border-box' },
        }}
      >
        <Toolbar />
        <Box sx={{ overflow: 'auto' }}>
          <List>
            {['Configurations', 'Data', 'Radar'].map((text, index) => (
              <ListItem key={text} disablePadding>
                <ListItemButton onClick={() => setSelectedItem(text)}>
                  <ListItemIcon>
                    {text === 'Configurations' && <SettingIcon />}
                    {text === 'Data' && <DataIcon />}
                    {text === 'Radar' && <RadarIcon />}

                  </ListItemIcon>
                  <ListItemText primary={text} />
                </ListItemButton>
              </ListItem>
            ))}
          </List>
          <Divider />

          <Stack spacing={2} padding={5}>
              <Button variant="contained" color="success" id='startButton' onClick={startButtonClickHandler} disabled={startDisabled}>
                START
              </Button>

              <Button variant="contained" color="error" onClick={stopButtonClickHandler} disabled={stopDisabled}>
                STOP
              </Button>
          </Stack>
        </Box>
      </Drawer>
      <Box component="main" sx={{ flexGrow: 1, p: 3 }}>
        <Toolbar />
        <SimpleLineChart distance={samples.distance} angle={samples.angle}/>
      </Box>
    </Box>
  );
}


export default AppDrawer;