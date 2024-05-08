import * as React from 'react';
import { LineChart } from '@mui/x-charts/LineChart';
import { Stack } from '@mui/system';
import { light } from 'ayu';
interface Data {
    distance: Array<number>;
    angle: Array<number>;
}

export default function DisplayData(data: Data) {
    return (
        <Stack spacing={2}>
            <LineChart
                width={500}
                height={300}
                colors={[light.syntax.entity.hex()]}
                series={[
                    { data: data.distance, label: 'Distance (mm)' }
                ]}
                xAxis={[{ scaleType: 'point', data: Array.from({ length: 50 }, (_, i) => i) }]}
            />

            <LineChart
                width={500}
                height={300}
                colors={[light.syntax.func.hex()]}
                series={[
                    { data: data.angle, label: 'Angle (°)' },
                ]}
                xAxis={[{ scaleType: 'point', data: Array.from({ length: 50 }, (_, i) => i) }]}
            />
        </Stack>

    );
}