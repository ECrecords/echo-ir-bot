import * as React from 'react';
import { LineChart } from '@mui/x-charts/LineChart';


interface Data {
    distance: Array<number>;
    angle: Array<number>;
}

export default function SimpleLineChart(data: Data) {
  return (
    <LineChart
      width={500}
      height={300}
      series={[
        { data: data.distance, label: 'Distance (mm)' },
        { data: data.angle, label: 'Angle (°)' },
      ]}
      xAxis={[{ scaleType: 'point', data: Array.from({length: 50}, (_, i) => i) }]}
    />
  );
}