pub mod kcp {
    use std::collections::VecDeque;

    pub struct IKCPSEG {
        pub conv: u32,
        pub cmd: u32,
        pub frg: u32,
        pub wnd: u32,
        pub ts: u32,
        pub sn: u32,
        pub una: u32,
        pub resendts: u32,
        pub rto: u32,
        pub fastack: u32,
        pub xmit: u32,
        pub data: Vec<u8>,
    }

    pub struct IKCPCB {
        pub conv: u32,
        pub mtu: u32,
        pub mss: u32,
        pub state: bool,

        pub snd_una: u32,
        pub snd_nxt: u32,
        pub rcv_nxt: u32,

        pub ts_recent: u32,
        pub ts_lastack: u32,
        pub ssthresh: u32,

        pub rx_rttval: i32,
        pub rx_srtt: i32,
        pub rx_rto: i32,
        pub rx_minrto: i32,

        pub snd_wnd: u32,
        pub rcv_wnd: u32,
        pub rmt_wnd: u32,
        pub cwnd: u32,
        pub probe: u32,

        pub current: u32,
        pub interval: u32,
        pub ts_flush: u32,
        pub xmit: u32,

        pub nodelay: u32,
        pub updated: bool,

        pub ts_probe: u32,
        pub probe_wait: u32,

        pub dead_link: u32,
        pub incr: u32,

        pub snd_queue: VecDeque<IKCPSEG>,
        pub rcv_queue: VecDeque<IKCPSEG>,
        pub snd_buf: VecDeque<IKCPSEG>,
        pub rcv_buf: VecDeque<IKCPSEG>,
        pub acklist: Vec<(u32, u32)>,
        pub fastresend: i32,
        pub fastlimit: i32,

        pub nocwnd: i32,
        pub stream: bool,
    }

    pub const IKCP_LOG_OUTPUT: u32 = 1;
    pub const IKCP_LOG_INPUT: u32 = 2;
    pub const IKCP_LOG_SEND: u32 = 4;
    pub const IKCP_LOG_RECV: u32 = 8;
    pub const IKCP_LOG_IN_DATA: u32 = 16;
    pub const IKCP_LOG_IN_ACK: u32 = 32;
    pub const IKCP_LOG_IN_PROBE: u32 = 64;
    pub const IKCP_LOG_IN_WINS: u32 = 128;
    pub const IKCP_LOG_OUT_DATA: u32 = 256;
    pub const IKCP_LOG_OUT_ACK: u32 = 512;
    pub const IKCP_LOG_OUT_PROBE: u32 = 1024;
    pub const IKCP_LOG_OUT_WINS: u32 = 2048;
}
