// RadarDisplay.tsx
import React, { useRef, useEffect } from 'react';

interface RadarDisplayProps {
    angle: number;    // Angle in degrees
    distance: number; // Distance as a percentage of the radar's radius
}

const RadarDisplay: React.FC<RadarDisplayProps> = ({ angle, distance }) => {
    const canvasRef = useRef<HTMLCanvasElement>(null);

    useEffect(() => {
        if (canvasRef.current) {
            const canvas = canvasRef.current;
            const context = canvas.getContext('2d');
            if (context) {
                drawRadar(context, angle, distance, canvas.width, canvas.height);
            }
        }
    }, [angle, distance]);

    // Draw function
    const drawRadar = (ctx: CanvasRenderingContext2D, angle: number, distance: number, width: number, height: number) => {
        const radius = Math.min(width, height) / 2; // Radar radius
        const xCenter = width / 2;
        const yCenter = height / 2;

        // Clear canvas
        ctx.clearRect(0, 0, width, height);

        // Draw background
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, width, height);

        // Draw circles
        ctx.strokeStyle = '#0F0';
        for (let i = 1; i <= 4; i++) {
            ctx.beginPath();
            ctx.arc(xCenter, yCenter, (radius / 5) * i, 0, 2 * Math.PI);
            ctx.stroke();
        }

        // Convert angle from degrees to radians
        const angleRad = (angle - 90) * (Math.PI / 180);

        // Calculate the position of the detected object
        const objectX = xCenter + distance * radius * Math.cos(angleRad);
        const objectY = yCenter + distance * radius * Math.sin(angleRad);

        // Draw line for angle
        ctx.beginPath();
        ctx.moveTo(xCenter, yCenter);
        ctx.lineTo(objectX, objectY);
        ctx.lineWidth = 2;
        ctx.stroke();

        // Draw object detection point
        ctx.fillStyle = '#F00';
        ctx.beginPath();
        ctx.arc(objectX, objectY, 5, 0, 2 * Math.PI);
        ctx.fill();
    };

    return (
        <canvas ref={canvasRef} width="300" height="300" style={{ background: '#333' }} />
    );
};

export default RadarDisplay;
