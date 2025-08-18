pub mod kcp {
    use crate::h::kcp::*;
    use std::collections::VecDeque;

    //=====================================================================
    // KCP BASIC
    //=====================================================================
    pub const IKCP_RTO_NDL: u32 = 30; // no delay min rto
    pub const IKCP_RTO_MIN: u32 = 100; // normal min rto
    pub const IKCP_RTO_DEF: u32 = 200;
    pub const IKCP_RTO_MAX: u32 = 60000;
    pub const IKCP_CMD_PUSH: u8 = 81; // cmd: push data
    pub const IKCP_CMD_ACK: u8 = 82; // cmd: ack
    pub const IKCP_CMD_WASK: u8 = 83; // cmd: window probe (ask)
    pub const IKCP_CMD_WINS: u8 = 84; // cmd: window size (tell)
    pub const IKCP_ASK_SEND: u32 = 1; // need to send IKCP_CMD_WASK
    pub const IKCP_ASK_TELL: u32 = 2; // need to send IKCP_CMD_WINS
    pub const IKCP_WND_SND: u32 = 32;
    pub const IKCP_WND_RCV: u32 = 128; // must >= max fragment size
    pub const IKCP_MTU_DEF: u32 = 1400;
    pub const IKCP_ACK_FAST: u32 = 3;
    pub const IKCP_INTERVAL: u32 = 100;
    pub const IKCP_OVERHEAD: u32 = 24;
    pub const IKCP_DEADLINK: u32 = 20;
    pub const IKCP_THRESH_INIT: u32 = 2;
    pub const IKCP_THRESH_MIN: u32 = 2;
    pub const IKCP_PROBE_INIT: u32 = 7000; // 7 secs to probe window size
    pub const IKCP_PROBE_LIMIT: u32 = 120000; // up to 120 secs to probe window
    pub const IKCP_FASTACK_LIMIT: u32 = 5; // max times to trigger fastack

    pub fn memcpy(dst: &mut [u8], src: &[u8], len: usize) {
        dst[..len].copy_from_slice(&src[..len]);
    }

    pub fn memoffset(dst: &mut [u8], src: usize) -> usize {
        (dst.as_ptr() as usize) - src
    }

    //---------------------------------------------------------------------
    // encode / decode
    //---------------------------------------------------------------------

    /* encode 8 bits unsigned int */
    #[inline]
    pub fn ikcp_encode8u(p: &mut [u8], c: u8) -> &mut [u8] {
        p[0] = c;
        &mut p[1..]
    }

    /* decode 8 bits unsigned int */
    #[inline]
    pub fn ikcp_decode8u<'a>(p: &'a [u8], c: &mut u8) -> &'a [u8] {
        *c = p[0];
        &p[1..]
    }

    /* encode 16 bits unsigned int (lsb) */
    #[inline]
    pub fn ikcp_encode16u(p: &mut [u8], w: u16) -> &mut [u8] {
        memcpy(p, &w.to_le_bytes(), 2);
        &mut p[2..]
    }

    /* decode 16 bits unsigned int (lsb) */
    #[inline]
    pub fn ikcp_decode16u<'a>(p: &'a [u8], w: &mut u16) -> &'a [u8] {
        *w = u16::from_le_bytes([p[0], p[1]]);
        &p[2..]
    }

    /* encode 32 bits unsigned int (lsb) */
    #[inline]
    pub fn ikcp_encode32u(p: &mut [u8], l: u32) -> &mut [u8] {
        memcpy(p, &l.to_le_bytes(), 4);
        &mut p[4..]
    }

    /* decode 32 bits unsigned int (lsb) */
    #[inline]
    pub fn ikcp_decode32u<'a>(p: &'a [u8], l: &mut u32) -> &'a [u8] {
        *l = u32::from_le_bytes([p[0], p[1], p[2], p[3]]);
        &p[4..]
    }

    #[inline]
    pub fn _imin_(a: u32, b: u32) -> u32 {
        a.min(b)
    }

    #[inline]
    pub fn _imax_(a: u32, b: u32) -> u32 {
        a.max(b)
    }

    #[inline]
    pub fn _ibound_(lower: u32, middle: u32, upper: u32) -> u32 {
        _imin_(_imax_(lower, middle), upper)
    }

    #[inline]
    pub fn _itimediff(later: u32, earlier: u32) -> i32 {
        later.wrapping_sub(earlier) as i32
    }

    // allocate a new kcp segment
    pub fn ikcp_segment_new(size: i32) -> IKCPSEG {
        IKCPSEG {
            conv: 0,
            cmd: 0,
            frg: 0,
            wnd: 0,
            ts: 0,
            sn: 0,
            una: 0,
            resendts: 0,
            rto: 0,
            fastack: 0,
            xmit: 0,
            data: vec![0; size as usize],
        }
    }

    // output segment
    pub fn ikcp_output<F>(
        kcp: &IKCPCB,
        size: i32,
        buffer: &mut [u8],
        destination: &mut [u8],
        output: &mut F,
    ) where
        F: FnMut(&IKCPCB, &mut [u8], &mut [u8], i32),
    {
        assert!(size >= 0);

        if size == 0 {
            return;
        }

        (*output)(kcp, buffer, destination, size);
    }

    //---------------------------------------------------------------------
    // create a new kcpcb
    //---------------------------------------------------------------------
    pub fn ikcp_create(conv: u32) -> IKCPCB {
        IKCPCB {
            conv,
            snd_una: 0,
            snd_nxt: 0,
            rcv_nxt: 0,
            ts_recent: 0,
            ts_lastack: 0,
            ts_probe: 0,
            probe_wait: 0,
            snd_wnd: IKCP_WND_SND,
            rcv_wnd: IKCP_WND_RCV,
            rmt_wnd: IKCP_WND_RCV,
            cwnd: 0,
            incr: 0,
            probe: 0,
            mtu: IKCP_MTU_DEF,
            mss: IKCP_MTU_DEF - IKCP_OVERHEAD,
            stream: false,
            snd_queue: VecDeque::new(),
            rcv_queue: VecDeque::new(),
            snd_buf: VecDeque::new(),
            rcv_buf: VecDeque::new(),
            state: false,
            acklist: vec![],
            rx_srtt: 0,
            rx_rttval: 0,
            rx_rto: IKCP_RTO_DEF as i32,
            rx_minrto: IKCP_RTO_MIN as i32,
            current: 0,
            interval: IKCP_INTERVAL,
            ts_flush: IKCP_INTERVAL,
            nodelay: 0,
            updated: false,
            ssthresh: IKCP_THRESH_INIT,
            fastresend: 0,
            fastlimit: IKCP_FASTACK_LIMIT as i32,
            nocwnd: 0,
            xmit: 0,
            dead_link: IKCP_DEADLINK,
        }
    }

    //---------------------------------------------------------------------
    // release a new kcpcb
    //---------------------------------------------------------------------
    pub fn ikcp_release(kcp: &mut IKCPCB) {
        *kcp = ikcp_create(kcp.conv);
    }

    //---------------------------------------------------------------------
    // user/upper level recv: returns size, returns below zero for EAGAIN
    //---------------------------------------------------------------------
    pub fn ikcp_recv(kcp: &mut IKCPCB, mut buffer: Option<&mut [u8]>, mut len: i32) -> i32 {
        let ispeek = len < 0;
        let peeksize: i32;
        let mut recover = false;

        if kcp.rcv_queue.is_empty() {
            return -1;
        }

        if len < 0 {
            len = -len;
        }

        peeksize = ikcp_peeksize(kcp);

        if peeksize < 0 {
            return -2;
        }

        if peeksize > len {
            return -3;
        }

        if kcp.rcv_queue.len() as u32 >= kcp.rcv_wnd {
            recover = true;
        }

        len = 0;
        while let Some(seg) = kcp.rcv_queue.front() {
            let fragment: i32;

            if let Some(buf) = buffer {
                memcpy(buf, &seg.data, seg.data.len());
                buffer = Some(&mut buf[..seg.data.len()]);
            }

            len += seg.data.len() as i32;
            fragment = seg.frg as i32;

            if !ispeek {
                kcp.rcv_queue.pop_front();
            }

            if fragment == 0 {
                break;
            }
        }

        assert_eq!(len, peeksize);

        // move available data from rcv_buf -> rcv_queue
        while let Some(seg) = kcp.rcv_buf.front() {
            if seg.sn == kcp.rcv_nxt && (kcp.rcv_queue.len() as u32) < kcp.rcv_wnd {
                let removed_seg = kcp.rcv_buf.pop_front().unwrap();
                kcp.rcv_queue.push_back(removed_seg);
                kcp.rcv_nxt += 1;
            } else {
                break;
            }
        }

        // fast recover
        if (kcp.rcv_queue.len() as u32) < kcp.rcv_wnd && recover {
            // ready to send back IKCP_CMD_WINS in ikcp_flush
            // tell remote my window size
            kcp.probe |= IKCP_ASK_TELL;
        }

        len
    }

    //---------------------------------------------------------------------
    // peek data size
    //---------------------------------------------------------------------
    pub fn ikcp_peeksize(kcp: &IKCPCB) -> i32 {
        let mut length = 0;

        let seg = match kcp.rcv_queue.front() {
            Some(front_seg) => front_seg,
            None => return -1,
        };

        if seg.frg == 0 {
            return seg.data.len() as i32;
        }

        if (kcp.rcv_queue.len() as u32) < seg.frg + 1 {
            return -1;
        }

        for seg in &kcp.rcv_queue {
            length += seg.data.len() as i32;
            if seg.frg == 0 {
                break;
            }
        }

        length
    }

    //---------------------------------------------------------------------
    // user/upper level send, returns below zero for error
    //---------------------------------------------------------------------
    pub fn ikcp_send(kcp: &mut IKCPCB, mut buffer: Option<&mut [u8]>, mut len: i32) -> i32 {
        let mut count: i32;

        let mut sent = 0;

        assert!(kcp.mss > 0);

        if len < 0 {
            return -1;
        }

        // append to previous segment in streaming mode (if possible)
        if kcp.stream {
            if let Some(old) = kcp.snd_queue.back() {
                if (old.data.len() as u32) < kcp.mss {
                    let capacity: i32 = (kcp.mss - old.data.len() as u32) as i32;
                    let extend = if len < capacity { len } else { capacity };
                    let mut seg = ikcp_segment_new((old.data.len() as i64 + extend as i64) as i32);

                    memcpy(&mut seg.data, &old.data, old.data.len());

                    if let Some(buf) = buffer {
                        memcpy(&mut seg.data[old.data.len()..], &buf, extend as usize);
                        buffer = Some(&mut buf[extend as usize..]);
                    }

                    seg.frg = 0;
                    kcp.snd_queue.push_back(seg);
                    len -= extend;
                    kcp.snd_queue.remove(kcp.snd_queue.len() - 1);
                    sent = extend;
                }

                if len <= 0 {
                    return sent;
                }
            }
        }

        if len <= kcp.mss as i32 {
            count = 1;
        } else {
            count = ((len as i64 + kcp.mss as i64 - 1) / kcp.mss as i64) as i32;
        }

        if !kcp.stream && count > 255 {
            return -2;
        }

        if count >= kcp.rcv_wnd as i32 {
            if kcp.stream && sent > 0 {
                return sent;
            }
            return -2;
        }

        if count == 0 {
            count = 1;
        }

        // fragment
        for i in 0..count {
            let size = if len > kcp.mss as i32 {
                kcp.mss as i32
            } else {
                len
            };

            let mut seg = ikcp_segment_new(size);

            if let Some(buf) = buffer {
                if size > 0 {
                    memcpy(&mut seg.data, buf, size as usize);
                }

                buffer = Some(&mut buf[..size as usize]);
            }

            seg.frg = if !kcp.stream {
                (count - i - 1) as u32
            } else {
                0
            };

            kcp.snd_queue.push_back(seg);

            len -= size;
            sent += size;
        }

        sent
    }

    //---------------------------------------------------------------------
    // parse ack
    //---------------------------------------------------------------------
    pub fn ikcp_update_ack(kcp: &mut IKCPCB, rtt: i32) {
        let rto;
        if kcp.rx_srtt == 0 {
            kcp.rx_srtt = rtt;
            kcp.rx_rttval = rtt / 2;
        } else {
            let mut delta: i64 = (rtt - kcp.rx_srtt) as i64;
            if delta < 0 {
                delta = -delta;
            }

            kcp.rx_rttval = ((3 * kcp.rx_rttval as i64 + delta) / 4) as i32;
            kcp.rx_srtt = (7 * kcp.rx_srtt + rtt) / 8;
            if kcp.rx_srtt < 1 {
                kcp.rx_srtt = 1;
            }
        }

        rto = (kcp.rx_srtt as i64 + _imax_(kcp.interval, (4 * kcp.rx_rttval) as u32) as i64) as i32;
        kcp.rx_rto = _ibound_(kcp.rx_minrto as u32, rto as u32, IKCP_RTO_MAX) as i32;
    }

    pub fn ikcp_shrink_buf(kcp: &mut IKCPCB) {
        if let Some(seg) = kcp.snd_buf.front() {
            kcp.snd_una = seg.sn;
        } else {
            kcp.snd_una = kcp.snd_nxt;
        }
    }

    pub fn ikcp_parse_ack(kcp: &mut IKCPCB, sn: u32) {
        if _itimediff(sn, kcp.snd_una) < 0 || _itimediff(sn, kcp.snd_nxt) >= 0 {
            return;
        }

        for i in 0..kcp.snd_buf.len() {
            let seg = &kcp.snd_buf[i];
            if sn == seg.sn {
                kcp.snd_buf.remove(i);
                break;
            }

            if _itimediff(sn, seg.sn) < 0 {
                break;
            }
        }
    }

    pub fn ikcp_parse_una(kcp: &mut IKCPCB, una: u32) {
        if let Some(i) = kcp
            .snd_buf
            .iter()
            .position(|seg| _itimediff(una, seg.sn) <= 0)
        {
            kcp.snd_buf.drain(0..i);
        }
    }

    pub fn ikcp_parse_fastack(kcp: &mut IKCPCB, sn: u32, ts: u32) {
        if _itimediff(sn, kcp.snd_una) < 0 || _itimediff(sn, kcp.snd_nxt) >= 0 {
            return;
        }

        for seg in &mut kcp.snd_buf {
            if _itimediff(sn, seg.sn) < 0 {
                break;
            }
            if sn != seg.sn && _itimediff(ts, seg.ts) >= 0 {
                if _itimediff(ts, seg.ts) >= 0 {
                    seg.fastack += 1;
                }
            }
        }
    }

    pub fn ikcp_ack_push(kcp: &mut IKCPCB, sn: u32, ts: u32) {
        kcp.acklist.push((sn, ts));
    }

    pub fn ikcp_ack_get(kcp: &IKCPCB, p: i32, sn: &mut u32, ts: &mut u32) {
        (*sn, *ts) = kcp.acklist[p as usize];
    }

    pub fn ikcp_parse_data(kcp: &mut IKCPCB, newseg: IKCPSEG) {
        let sn = newseg.sn;
        let mut repeat = false;

        let mut insert_position = None;

        if _itimediff(sn, kcp.rcv_nxt + kcp.rcv_wnd) >= 0 || _itimediff(sn, kcp.rcv_nxt) < 0 {
            return;
        }

        for (i, seg) in kcp.rcv_buf.iter().enumerate().rev() {
            if seg.sn == sn {
                repeat = true;
                break;
            }

            if _itimediff(sn, seg.sn) > 0 {
                insert_position = Some(i + 1);
                break;
            }
        }

        if !repeat {
            match insert_position {
                Some(position) => kcp.rcv_buf.insert(position, newseg),
                None => kcp.rcv_buf.push_front(newseg),
            }
        }

        // move available data from rcv_buf -> rcv_queue
        while let Some(seg) = kcp.rcv_buf.front() {
            if seg.sn == kcp.rcv_nxt && (kcp.rcv_queue.len() as u32) < kcp.rcv_wnd {
                let removed_seg = kcp.rcv_buf.pop_front().unwrap();
                kcp.rcv_queue.push_back(removed_seg);
                kcp.rcv_nxt += 1;
            } else {
                break;
            }
        }
    }

    //---------------------------------------------------------------------
    // input data
    //---------------------------------------------------------------------
    pub fn ikcp_input(kcp: &mut IKCPCB, mut data: &[u8], mut size: i64) -> i32 {
        let prev_una = kcp.snd_una;

        let mut maxack: u32 = 0;
        let mut latest_ts: u32 = 0;

        let mut flag = 0;

        if size < IKCP_OVERHEAD as i64 {
            return -1;
        }

        let mut ts: u32 = 0;
        let mut sn: u32 = 0;
        let mut len: u32 = 0;
        let mut una: u32 = 0;
        let mut conv: u32 = 0;

        let mut wnd: u16 = 0;
        let mut cmd: u8 = 0;
        let mut frg: u8 = 0;

        loop {
            let mut seg: IKCPSEG;

            if size < IKCP_OVERHEAD as i64 {
                break;
            }

            data = ikcp_decode32u(data, &mut conv);

            if conv != kcp.conv {
                return -1;
            }

            data = ikcp_decode8u(data, &mut cmd);
            data = ikcp_decode8u(data, &mut frg);
            data = ikcp_decode16u(data, &mut wnd);
            data = ikcp_decode32u(data, &mut ts);
            data = ikcp_decode32u(data, &mut sn);
            data = ikcp_decode32u(data, &mut una);
            data = ikcp_decode32u(data, &mut len);

            size -= IKCP_OVERHEAD as i64;

            if size < (len as i64) || (len as i32) < 0 {
                return -2;
            }

            if cmd != IKCP_CMD_PUSH
                && cmd != IKCP_CMD_ACK
                && cmd != IKCP_CMD_WASK
                && cmd != IKCP_CMD_WINS
            {
                return -3;
            }

            kcp.rmt_wnd = wnd as u32;
            ikcp_parse_una(kcp, una);
            ikcp_shrink_buf(kcp);

            match cmd {
                IKCP_CMD_ACK => {
                    if _itimediff(kcp.current, ts) >= 0 {
                        ikcp_update_ack(kcp, _itimediff(kcp.current, ts));
                    }

                    ikcp_parse_ack(kcp, sn);
                    ikcp_shrink_buf(kcp);
                    if flag == 0 {
                        flag = 1;
                        maxack = sn;
                        latest_ts = ts;
                    } else {
                        if _itimediff(sn, maxack) > 0 {
                            if _itimediff(ts, latest_ts) > 0 {
                                maxack = sn;
                                latest_ts = ts;
                            }
                        }
                    }
                }

                IKCP_CMD_PUSH => {
                    if _itimediff(sn, kcp.rcv_nxt + kcp.rcv_wnd) < 0 {
                        ikcp_ack_push(kcp, sn, ts);
                        if _itimediff(sn, kcp.rcv_nxt) >= 0 {
                            seg = ikcp_segment_new(len as i32);
                            seg.conv = conv;
                            seg.cmd = cmd as u32;
                            seg.frg = frg as u32;
                            seg.wnd = wnd as u32;
                            seg.ts = ts;
                            seg.sn = sn;
                            seg.una = una;

                            if len > 0 {
                                memcpy(&mut seg.data, &data, len as usize);
                            }

                            ikcp_parse_data(kcp, seg);
                        }
                    }
                }

                IKCP_CMD_WASK => {
                    // ready to send back IKCP_CMD_WINS in ikcp_flush
                    // tell remote my window size
                    kcp.probe |= IKCP_ASK_TELL;
                }

                IKCP_CMD_WINS => {
                    // do nothing
                }

                _ => return -3,
            }

            data = &data[len as usize..];
            size -= len as i64;
        }

        if flag != 0 {
            ikcp_parse_fastack(kcp, maxack, latest_ts);
        }

        if _itimediff(kcp.snd_una, prev_una) > 0 {
            if kcp.cwnd < kcp.rmt_wnd {
                let mss = kcp.mss;
                if kcp.cwnd < kcp.ssthresh {
                    kcp.cwnd += 1;
                    kcp.incr += mss;
                } else {
                    if kcp.incr < mss {
                        kcp.incr = mss;
                    }
                    kcp.incr += (mss * mss) / kcp.incr + (mss / 16);
                    if (kcp.cwnd + 1) * mss <= kcp.incr {
                        kcp.cwnd = (kcp.incr + mss - 1) / (if mss > 0 { mss } else { 1 });
                    }
                }

                if kcp.cwnd > kcp.rmt_wnd {
                    kcp.cwnd = kcp.rmt_wnd;
                    kcp.incr = kcp.rmt_wnd * mss;
                }
            }
        }

        0
    }

    //---------------------------------------------------------------------
    // ikcp_encode_seg
    //---------------------------------------------------------------------
    pub fn ikcp_encode_seg<'a>(mut ptr: &'a mut [u8], seg: &IKCPSEG) -> &'a mut [u8] {
        ptr = ikcp_encode32u(ptr, seg.conv);
        ptr = ikcp_encode8u(ptr, seg.cmd as u8);
        ptr = ikcp_encode8u(ptr, seg.frg as u8);
        ptr = ikcp_encode16u(ptr, seg.wnd as u16);
        ptr = ikcp_encode32u(ptr, seg.ts);
        ptr = ikcp_encode32u(ptr, seg.sn);
        ptr = ikcp_encode32u(ptr, seg.una);
        ptr = ikcp_encode32u(ptr, seg.data.len() as u32);
        ptr
    }

    pub fn ikcp_wnd_unused(kcp: &IKCPCB) -> i32 {
        if (kcp.rcv_queue.len() as u32) < kcp.rcv_wnd {
            return (kcp.rcv_wnd - (kcp.rcv_queue.len() as u32)) as i32;
        }

        0
    }

    //---------------------------------------------------------------------
    // ikcp_flush
    //---------------------------------------------------------------------
    pub fn ikcp_flush<F>(
        kcp: &mut IKCPCB,
        buffer: &mut [u8],
        destination: &mut [u8],
        output: &mut F,
    ) where
        F: FnMut(&IKCPCB, &mut [u8], &mut [u8], i32),
    {
        let current = kcp.current;
        let position = buffer.as_ptr() as usize;
        let mut ptr = &mut *buffer;

        let mut size: i32;

        let resent: u32;
        let mut cwnd: u32;

        let rtomin: u32;

        let mut change = 0;
        let mut lost = false;

        let mut seg: IKCPSEG;

        // 'ikcp_update' haven't been called.
        if !kcp.updated {
            return;
        }

        seg = IKCPSEG {
            conv: kcp.conv,
            cmd: IKCP_CMD_ACK as u32,
            frg: 0,
            wnd: ikcp_wnd_unused(kcp) as u32,
            ts: 0,
            sn: 0,
            una: kcp.rcv_nxt,
            resendts: 0,
            rto: 0,
            fastack: 0,
            xmit: 0,
            data: vec![],
        };

        // flush acknowledges
        for i in 0..(kcp.acklist.len() as i32) {
            size = memoffset(ptr, position) as i32;
            if size + IKCP_OVERHEAD as i32 > kcp.mtu as i32 {
                ikcp_output(kcp, size, buffer, destination, output);
                ptr = &mut *buffer;
            }

            ikcp_ack_get(kcp, i, &mut seg.sn, &mut seg.ts);
            ptr = ikcp_encode_seg(ptr, &seg);
        }

        kcp.acklist.clear();

        // probe window size (if remote window size equals zero)
        if kcp.rmt_wnd == 0 {
            if kcp.probe_wait == 0 {
                kcp.probe_wait = IKCP_PROBE_INIT;
                kcp.ts_probe = kcp.current + kcp.probe_wait;
            } else {
                if _itimediff(kcp.current, kcp.ts_probe) >= 0 {
                    if kcp.probe_wait < IKCP_PROBE_INIT {
                        kcp.probe_wait = IKCP_PROBE_INIT;
                    }
                    kcp.probe_wait += kcp.probe_wait / 2;
                    if kcp.probe_wait > IKCP_PROBE_LIMIT {
                        kcp.probe_wait = IKCP_PROBE_LIMIT;
                    }
                    kcp.ts_probe = kcp.current + kcp.probe_wait;
                    kcp.probe |= IKCP_ASK_SEND;
                }
            }
        } else {
            kcp.ts_probe = 0;
            kcp.probe_wait = 0;
        }

        // flush window probing commands
        if (kcp.probe & IKCP_ASK_SEND) != 0 {
            seg.cmd = IKCP_CMD_WASK as u32;
            size = memoffset(ptr, position) as i32;
            if size + IKCP_OVERHEAD as i32 > kcp.mtu as i32 {
                ikcp_output(kcp, size, buffer, destination, output);
                ptr = &mut *buffer;
            }

            ptr = ikcp_encode_seg(ptr, &seg);
        }

        // flush window probing commands
        if (kcp.probe & IKCP_ASK_TELL) != 0 {
            seg.cmd = IKCP_CMD_WINS as u32;
            size = memoffset(ptr, position) as i32;
            if size + IKCP_OVERHEAD as i32 > kcp.mtu as i32 {
                ikcp_output(kcp, size, buffer, destination, output);
                ptr = &mut *buffer;
            }

            ptr = ikcp_encode_seg(ptr, &seg);
        }

        kcp.probe = 0;

        // calculate window size
        cwnd = _imin_(kcp.snd_wnd, kcp.rmt_wnd);
        if kcp.nocwnd == 0 {
            cwnd = _imin_(kcp.cwnd, cwnd);
        }

        // move data from snd_queue to snd_buf
        while _itimediff(kcp.snd_nxt, kcp.snd_una + cwnd) < 0 {
            if let Some(mut newseg) = kcp.snd_queue.pop_front() {
                newseg.conv = kcp.conv;
                newseg.cmd = IKCP_CMD_PUSH as u32;
                newseg.wnd = seg.wnd;
                newseg.ts = current;
                newseg.sn = kcp.snd_nxt;
                kcp.snd_nxt += 1;
                newseg.una = kcp.rcv_nxt;
                newseg.resendts = current;
                newseg.rto = kcp.rx_rto as u32;
                newseg.fastack = 0;
                newseg.xmit = 0;
                kcp.snd_buf.push_back(newseg);
            } else {
                break;
            }
        }

        // calculate resent
        resent = if kcp.fastresend > 0 {
            kcp.fastresend as u32
        } else {
            0xffffffff
        };

        rtomin = if kcp.nodelay == 0 {
            (kcp.rx_rto >> 3) as u32
        } else {
            0
        };

        // flush data segments
        for i in 0..kcp.snd_buf.len() {
            let mut segment = &mut kcp.snd_buf[i];
            let mut needsend = false;
            if segment.xmit == 0 {
                needsend = true;
                segment.xmit += 1;
                segment.rto = kcp.rx_rto as u32;
                segment.resendts = current + segment.rto + rtomin;
            } else if _itimediff(current, segment.resendts) >= 0 {
                needsend = true;
                segment.xmit += 1;
                kcp.xmit += 1;
                if kcp.nodelay == 0 {
                    segment.rto += _imax_(segment.rto, kcp.rx_rto as u32);
                } else {
                    let step = if kcp.nodelay < 2 {
                        segment.rto as i32
                    } else {
                        kcp.rx_rto
                    };
                    segment.rto += (step / 2) as u32;
                }

                segment.resendts = current + segment.rto;
                lost = true;
            } else if segment.fastack >= resent {
                if segment.xmit <= kcp.fastlimit as u32 || kcp.fastlimit <= 0 {
                    needsend = true;
                    segment.xmit += 1;
                    segment.fastack = 0;
                    segment.resendts = current + segment.rto;
                    change += 1;
                }
            }
            if needsend {
                let need;
                segment.ts = current;
                segment.wnd = seg.wnd;
                segment.una = kcp.rcv_nxt;

                size = memoffset(ptr, position) as i32;
                need = IKCP_OVERHEAD as i32 + segment.data.len() as i32;

                if (size + need) > (kcp.mtu as i32) {
                    ikcp_output(kcp, size, buffer, destination, output);
                    ptr = &mut *buffer;

                    segment = &mut kcp.snd_buf[i];
                }

                ptr = ikcp_encode_seg(ptr, &segment);

                if segment.data.len() > 0 {
                    memcpy(ptr, &segment.data, segment.data.len());
                    ptr = &mut ptr[segment.data.len()..];
                }

                if segment.xmit >= kcp.dead_link {
                    kcp.state = true;
                }
            }
        }

        // flush remain segments
        size = memoffset(ptr, position) as i32;
        if size > 0 {
            ikcp_output(kcp, size, buffer, destination, output);
        }

        // update ssthresh
        if change != 0 {
            let inflight = kcp.snd_nxt - kcp.snd_una;
            kcp.ssthresh = inflight / 2;
            if kcp.ssthresh < IKCP_THRESH_MIN {
                kcp.ssthresh = IKCP_THRESH_MIN;
            }
            kcp.cwnd = kcp.ssthresh + resent;
            kcp.incr = kcp.cwnd * kcp.mss;
        }

        if lost {
            kcp.ssthresh = cwnd / 2;
            if kcp.ssthresh < IKCP_THRESH_MIN {
                kcp.ssthresh = IKCP_THRESH_MIN;
            }
            kcp.cwnd = 1;
            kcp.incr = kcp.mss;
        }

        if kcp.cwnd < 1 {
            kcp.cwnd = 1;
            kcp.incr = kcp.mss;
        }
    }

    pub fn ikcp_update<F>(
        kcp: &mut IKCPCB,
        current: u32,
        buffer: &mut [u8],
        destination: &mut [u8],
        output: &mut F,
    ) where
        F: FnMut(&IKCPCB, &mut [u8], &mut [u8], i32),
    {
        let mut slap: i32;

        kcp.current = current;

        if !kcp.updated {
            kcp.updated = true;
            kcp.ts_flush = kcp.current;
        }

        slap = _itimediff(kcp.current, kcp.ts_flush);

        if slap >= 10000 || slap < -10000 {
            kcp.ts_flush = kcp.current;
            slap = 0;
        }

        if slap >= 0 {
            kcp.ts_flush += kcp.interval;
            if _itimediff(kcp.current, kcp.ts_flush) >= 0 {
                kcp.ts_flush = kcp.current + kcp.interval;
            }

            ikcp_flush(kcp, buffer, destination, output);
        }
    }

    //---------------------------------------------------------------------
    // Determine when should you invoke ikcp_update:
    // returns when you should invoke ikcp_update in millisec, if there
    // is no ikcp_input/_send calling. you can call ikcp_update in that
    // time, instead of call update repeatly.
    // Important to reduce unnacessary ikcp_update invoking. use it to
    // schedule ikcp_update (for example. implementing an epoll-like mechanism,
    // or optimize ikcp_update when handling massive kcp connections)
    //---------------------------------------------------------------------
    pub fn ikcp_check(kcp: &IKCPCB, current: u32) -> u32 {
        let mut ts_flush = kcp.ts_flush;
        let tm_flush;
        let mut tm_packet = i32::MAX;
        let mut minimal;

        if !kcp.updated {
            return current;
        }

        if _itimediff(current, ts_flush) >= 10000 || _itimediff(current, ts_flush) < -10000 {
            ts_flush = current;
        }

        if _itimediff(current, ts_flush) >= 0 {
            return current;
        }

        tm_flush = _itimediff(ts_flush, current);

        for seg in &kcp.snd_buf {
            let diff = _itimediff(seg.resendts, current);
            if diff <= 0 {
                return current;
            }
            if diff < tm_packet {
                tm_packet = diff;
            }
        }

        minimal = if tm_packet < tm_flush {
            tm_packet
        } else {
            tm_flush
        } as u32;

        if minimal >= kcp.interval {
            minimal = kcp.interval;
        }

        current + minimal
    }

    pub fn ikcp_setmtu(kcp: &mut IKCPCB, mtu: i32) -> i32 {
        if mtu < 50 || mtu < IKCP_OVERHEAD as i32 {
            return -1;
        }

        kcp.mtu = mtu as u32;
        kcp.mss = kcp.mtu - IKCP_OVERHEAD;
        0
    }

    pub fn ikcp_interval(kcp: &mut IKCPCB, interval: i32) {
        let interval = if interval > 5000 {
            5000
        } else if interval < 10 {
            10
        } else {
            interval
        };
        kcp.interval = interval as u32;
    }

    pub fn ikcp_nodelay(kcp: &mut IKCPCB, nodelay: i32, mut interval: i32, resend: i32, nc: i32) {
        if nodelay >= 0 {
            kcp.nodelay = nodelay as u32;
            kcp.rx_minrto = if nodelay != 0 {
                IKCP_RTO_NDL as i32
            } else {
                IKCP_RTO_MIN as i32
            };
        }

        if interval >= 0 {
            if interval > 5000 {
                interval = 5000;
            } else if interval < 10 {
                interval = 10;
            }
            kcp.interval = interval as u32;
        }

        if resend >= 0 {
            kcp.fastresend = resend;
        }

        if nc >= 0 {
            kcp.nocwnd = nc;
        }
    }

    pub fn ikcp_wndsize(kcp: &mut IKCPCB, sndwnd: i32, rcvwnd: i32) {
        if sndwnd > 0 {
            kcp.snd_wnd = sndwnd as u32;
        }

        if rcvwnd > 0 {
            // must >= max fragment size
            kcp.rcv_wnd = _imax_(rcvwnd as u32, IKCP_WND_RCV);
        }
    }

    pub fn ikcp_waitsnd(kcp: &IKCPCB) -> i32 {
        (kcp.snd_buf.len() + kcp.snd_queue.len()) as i32
    }

    // read conv
    pub fn ikcp_getconv(kcp: &IKCPCB) -> u32 {
        kcp.conv
    }
}
