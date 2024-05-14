//! \file main.rs
//! \author Elvis Chino-Islas
//! \date May 4, 2024
//! 
//! \brief Radar Data Receiver and Server for PicoW.
//!
//! This Rust application serves a dual purpose: it listens for UDP packets containing
//! radar data (distance and angle) from a device known as PicoW, and serves this data
//! over HTTP using the Axum framework. It utilizes asynchronous programming principles
//! provided by Tokio and handles CORS configurations for web service interoperability.
//!
//! The program structures incoming radar data into two VecDeques within a shared,
//! mutex-protected state, ensuring thread-safe operations. It provides an HTTP endpoint
//! to access the latest radar data in JSON format, demonstrating basic real-time data
//! serving techniques in a Rust-based web application.

use std::sync::{Arc, Mutex};
use axum::{extract::State, http::Method, routing::get, Json, Router};
use tower_http::cors::{Any, CorsLayer};

use serde::Serialize;

use std::collections::VecDeque;
use tokio::net::UdpSocket;

/// Structure to hold radar samples with distances and angles.
#[derive(Debug, Serialize, Clone)]
struct Samples {
    distance: VecDeque<u32>,
    angle: VecDeque<u32>,
}

/// Maximum capacity of the VecDeque for samples.
const DEQUE_CAPACITY: usize = 50;

/// Address for receiving data from PicoW.
const PICOW_ADDR: &str = "0.0.0.0:8008";

/// Address for the backend to listen for HTTP requests.
const BACKEND_ADDR: &str = "0.0.0.0:3000";

/// Listens for incoming UDP packets from PicoW and updates running samples.
/// 
/// # Arguments
/// * `addr` - The socket address to bind to.
/// * `running_samples` - Shared state for storing radar samples.
async fn picow_listener(addr: String, running_samples: Arc<Mutex<Samples>>) {
    let Ok(socket) = UdpSocket::bind(addr).await else {
        panic!("Could not bind to address");
    };

    loop {
        let mut buffer = [0; 8];
        let _ = socket.recv_from(&mut buffer).await.unwrap();

        let distance = u32::from_be_bytes([buffer[7], buffer[6], buffer[5], buffer[4]]);
        let angle = u32::from_be_bytes([buffer[3], buffer[2], buffer[1], buffer[0]]);

        // Update shared state with the new samples
        if let Ok(mut lock) = running_samples.lock() {
            if lock.distance.len() == DEQUE_CAPACITY {
                lock.distance.pop_front();
            }

            if lock.angle.len() == DEQUE_CAPACITY {
                lock.angle.pop_front();
            }

            lock.distance.push_back(distance);
            lock.angle.push_back(angle);
        }

        // Artificial delay for rate limiting
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    }
}

/// Endpoint to get current radar samples as JSON.
/// 
/// Wraps the shared state in a JSON response.
async fn data(State(samples): State<Arc<Mutex<Samples>>>) -> Json<Samples> {
    let lock = samples.lock().unwrap();
    
    Json(Samples {
        distance: lock.distance.clone(),
        angle: lock.angle.clone(),
    })
}

/// Configures the web application with routes and CORS settings.
/// 
/// # Arguments
/// * `running_samples` - Shared state for storing radar samples.
fn app(running_samples: Arc<Mutex<Samples>>) -> Router {

    let cors = CorsLayer::new()
    .allow_methods(Method::GET)
    .allow_origin(Any);


    let app = Router::new()
        .route("/", get(|| async { "Hello, World!" }))
        .route("/data", get(data))
        .with_state(running_samples)
        .layer(cors);

    app
}

/// Main entry point for the application.
/// 
/// Initializes the application and spawns the UDP listener and HTTP server.
#[tokio::main]
async fn main() {
    // build our application with a single route

    // create a shared buffer to store picow data
    let buffer = Arc::new(Mutex::new(Samples {
        distance: VecDeque::with_capacity(DEQUE_CAPACITY),
        angle: VecDeque::with_capacity(DEQUE_CAPACITY),
    }));

    // run our app with hyper, listening globally on port 3000
    tokio::spawn(picow_listener(PICOW_ADDR.to_string(), buffer.clone()));
    let listener = tokio::net::TcpListener::bind(BACKEND_ADDR).await.unwrap();
    axum::serve(listener, app(buffer)).await.unwrap();
}
