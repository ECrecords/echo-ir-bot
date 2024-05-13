import React, { useRef, useEffect } from 'react';
import { light, dark } from 'ayu';
interface RadarDisplayProps {
    distance: number[];
    angle: number[];
}

const RadarDisplay: React.FC<RadarDisplayProps> = ({ distance, angle }) => {
    const canvasRef = useRef<HTMLCanvasElement>(null);

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const context = canvas.getContext('2d');
        if (!context) return;

        // Responsive canvas size
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width;
        canvas.height = rect.height;

        const width = canvas.width;
        const height = canvas.height;
        const centerX = width / 2;
        const centerY = height / 2;
        const maxRadarRange = Math.min(width, height) / 2 - 20; // 20 pixels padding

        context.clearRect(0, 0, width, height);

        // Radar background
        context.fillStyle = '#0000'; // light blue background
        context.fillRect(0, 0, width, height);

        // Draw concentric circles
        for (let i = 1; i <= 5; i++) {
            context.beginPath();
            context.arc(centerX, centerY, (i * maxRadarRange) / 5, 0, 2 * Math.PI);
            context.strokeStyle = '#bbb';
            context.stroke();
        }

        // Draw radial lines
        for (let i = 0; i < 360; i += 30) {
            context.beginPath();
            context.moveTo(centerX, centerY);
            const x = centerX + maxRadarRange * Math.cos((i * Math.PI) / 180);
            const y = centerY + maxRadarRange * Math.sin((i * Math.PI) / 180);
            context.lineTo(x, y);
            context.stroke();
        }

        // Draw each sample as a point
        distance.forEach((dist, index) => {
            const angleRad = (angle[index] * Math.PI) / 180; // Convert degrees to radians

            // Normalize the distance so it fits within the radar
            const normalizedDistance = (dist / Math.max(...distance)) * maxRadarRange;

            const x = centerX + normalizedDistance * Math.cos(angleRad);
            const y = centerY + normalizedDistance * Math.sin(angleRad);

            context.beginPath();
            context.arc(x, y, 5, 0, 2 * Math.PI); // Draw small circle for each point
            context.fillStyle = light.syntax.func.hex(); // Constant color, adjust alpha or color if needed
            context.fill();
        });
    }, [distance, angle]);

    return (
        <div style={{ width: '100%', height: '500px' }}>
            <canvas ref={canvasRef} style={{ width: '100%', height: '100%' }} />
        </div>
    );
};

export default RadarDisplay;
