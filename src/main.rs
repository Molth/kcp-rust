use kcp_rust::kcp::*;
use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let client_addr: SocketAddr = "127.0.0.1:10000".parse().unwrap();
    let server_addr: SocketAddr = "127.0.0.1:10001".parse().unwrap();

    let server_thread = thread::spawn(move || {
        kcp_server(server_addr);
    });

    thread::sleep(Duration::from_millis(100));

    let client_thread = thread::spawn(move || {
        kcp_client(client_addr, server_addr);
    });

    client_thread.join().unwrap();
    server_thread.join().unwrap();
}

fn kcp_server(addr: SocketAddr) {
    let socket = Arc::new(Mutex::new(
        UdpSocket::bind(addr).expect("Failed to bind server socket"),
    ));

    socket.lock().unwrap().set_nonblocking(true).unwrap();

    let mut kcp = ikcp_create(114514);

    ikcp_nodelay(&mut kcp, 1, 10, 1, true);

    let mut buffer = vec![0u8; 1500];

    let mut recv_buf = vec![0u8; 1500];
    let mut last_update = Instant::now();
    let start_time = Instant::now();

    println!("[Server] Listening on {}", addr);

    let mut remote_addr: Option<SocketAddr> = None;

    loop {
        match socket.lock().unwrap().recv_from(&mut recv_buf) {
            Ok((size, src)) => {
                remote_addr = Some(src);

                ikcp_input(&mut kcp, &recv_buf[..size], size as i64);
            }

            Err(ref e) if e.kind() == ErrorKind::WouldBlock => {}

            Err(e) => {
                eprintln!("[Server] UDP recv error: {}", e);
            }
        }

        let current = start_time.elapsed().as_millis() as u32;
        if last_update.elapsed() >= Duration::from_millis(10) {
            if remote_addr.is_some() {
                let mut output_closure = |buf: &mut [u8], len: i32, _: &IKCPCB, _: &mut ()| {
                    if len > 0 {
                        let socket = socket.lock().unwrap();
                        socket
                            .send_to(&buf[..len as usize], remote_addr.unwrap())
                            .unwrap();
                    }
                };

                ikcp_update(&mut kcp, current, &mut buffer, &mut (), &mut output_closure);
            }

            last_update = Instant::now();
        }

        let mut recv_buffer = [0u8; 1024];
        let size = ikcp_recv(&mut kcp, Some(&mut recv_buffer), 1024);
        if size > 0 {
            let msg = String::from_utf8_lossy(&recv_buffer[..size as usize]);
            println!("[Server] Received: {}", msg);

            let echo_msg = format!("ECHO: {}", msg);
            let mut data = echo_msg.as_bytes().to_vec();

            let len = data.len();
            let result = ikcp_send(&mut kcp, Some(&mut data), len as i32);
            println!("[Server] Echoed: {} (result: {})", echo_msg, result);
        } else if size != -1 {
            println!("[Server] Received Error: {}", size);
        }

        thread::sleep(Duration::from_millis(1));
    }
}

fn kcp_client(addr: SocketAddr, server_addr: SocketAddr) {
    let socket = Arc::new(Mutex::new(
        UdpSocket::bind(addr).expect("Failed to bind client socket"),
    ));

    socket.lock().unwrap().connect(server_addr).unwrap();
    socket.lock().unwrap().set_nonblocking(true).unwrap();

    let mut kcp = ikcp_create(114514);
    ikcp_nodelay(&mut kcp, 1, 10, 1, true);

    let mut buffer = vec![0u8; 1500];
    let mut recv_buf = vec![0u8; 1500];
    let mut last_update = Instant::now();
    let start_time = Instant::now();
    let mut last_send = Instant::now();
    let mut sequence = 0;

    println!("[Client] Connecting to {}", server_addr);
    let mut user = (0, vec![0u8; 1500]);

    loop {
        match socket.lock().unwrap().recv(&mut recv_buf) {
            Ok(size) => {
                ikcp_input(&mut kcp, &recv_buf[..size], size as i64);
            }

            Err(ref e) if e.kind() == ErrorKind::WouldBlock => {}

            Err(e) => {
                eprintln!("[Client] UDP recv error: {}", e);
            }
        }

        let current = start_time.elapsed().as_millis() as u32;
        if last_update.elapsed() >= Duration::from_millis(10) {
            let mut output_closure =
                |buf: &mut [u8], len: i32, _: &IKCPCB, user: &mut (i32, Vec<u8>)| {
                    if len > 0 {
                        user.0 += 1;
                        let socket = socket.lock().unwrap();
                        socket.send_to(&buf[..len as usize], server_addr).unwrap();
                    }
                };

            ikcp_update(
                &mut kcp,
                current,
                &mut buffer,
                &mut user,
                &mut output_closure,
            );

            last_update = Instant::now();
        }

        if last_send.elapsed() >= Duration::from_secs(1) {
            sequence += 1;
            let msg = format!("Hello KCP! {}", sequence);
            let mut data = msg.as_bytes().to_vec();
            let len = data.len() as i32;
            ikcp_send(&mut kcp, Some(&mut data), len);

            last_send = Instant::now();
        }

        let mut recv_buffer = [0u8; 1024];
        let size = ikcp_recv(&mut kcp, Some(&mut recv_buffer), 1024);
        if size > 0 {
            let msg = String::from_utf8_lossy(&recv_buffer[..size as usize]);
            println!("[Client] Received: {}", msg);
        }

        thread::sleep(Duration::from_millis(1));
    }
}
