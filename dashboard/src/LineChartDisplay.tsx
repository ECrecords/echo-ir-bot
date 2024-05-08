import React from 'react';
import { LineChart } from '@mui/x-charts/LineChart';

// hardcoded data for the chart
const pData = [
    10, 20, 30, 40, 50, 60,
];

const uData = [
    50, 100, 75, 200, 125, 250,
];

const xLabels = ['2015', '2016', '2017', '2018', '2019', '2020'];

export default function LineChartDisplay() {
    return <LineChart
        series={[
            { data: pData, label: 'pv' },
            { data: uData, label: 'uv' },
        ]}
        xAxis={[{ scaleType: 'point', data: xLabels }]}
    />
}