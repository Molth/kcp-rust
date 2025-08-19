use std::collections::VecDeque;

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

pub(crate) fn memcpy(dst: &mut [u8], src: &[u8], len: usize) {
    dst[..len].copy_from_slice(&src[..len]);
}

pub(crate) fn memoffset(dst: &mut [u8], src: usize) -> usize {
    (dst.as_ptr() as usize) - src
}

// encode 8 bits unsigned int
pub fn ikcp_encode8u(p: &mut [u8], c: u8) -> &mut [u8] {
    p[0] = c;
    &mut p[1..]
}

// decode 8 bits unsigned int
pub fn ikcp_decode8u<'a>(p: &'a [u8], c: &mut u8) -> &'a [u8] {
    *c = p[0];
    &p[1..]
}

// encode 16 bits unsigned int (lsb)
pub fn ikcp_encode16u(p: &mut [u8], w: u16) -> &mut [u8] {
    memcpy(p, &w.to_le_bytes(), 2);
    &mut p[2..]
}

// decode 16 bits unsigned int (lsb)
pub fn ikcp_decode16u<'a>(p: &'a [u8], w: &mut u16) -> &'a [u8] {
    *w = u16::from_le_bytes([p[0], p[1]]);
    &p[2..]
}

// encode 32 bits unsigned int (lsb)
pub fn ikcp_encode32u(p: &mut [u8], l: u32) -> &mut [u8] {
    memcpy(p, &l.to_le_bytes(), 4);
    &mut p[4..]
}

// decode 32 bits unsigned int (lsb)
pub fn ikcp_decode32u<'a>(p: &'a [u8], l: &mut u32) -> &'a [u8] {
    *l = u32::from_le_bytes([p[0], p[1], p[2], p[3]]);
    &p[4..]
}

pub fn ikcp_timediff(later: u32, earlier: u32) -> i32 {
    later.wrapping_sub(earlier) as i32
}

// allocate a new kcp segment
pub(crate) fn ikcp_segment_new(size: i32) -> IKCPSEG {
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
pub(crate) fn ikcp_output<T, F>(
    kcp: &IKCPCB,
    size: i32,
    data: &mut [u8],
    user: &mut T,
    output: &mut F,
) where
    F: FnMut(&mut [u8], i32, &IKCPCB, &mut T),
{
    if size == 0 {
        return;
    }

    (*output)(data, size, kcp, user);
}

// create a new kcp control object, 'conv' must equal in two endpoint
pub fn ikcp_create(conv: u32) -> IKCPCB {
    IKCPCB {
        conv,
        snd_una: 0,
        snd_nxt: 0,
        rcv_nxt: 0,
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
        nocwnd: false,
        xmit: 0,
        dead_link: IKCP_DEADLINK,
    }
}

// user/upper level recv: returns size, returns below zero for EAGAIN
pub fn ikcp_recv(kcp: &mut IKCPCB, mut buffer: Option<&mut [u8]>, mut len: i32) -> i32 {
    let ispeek = len < 0;
    let mut recover = false;

    if kcp.rcv_queue.is_empty() {
        return -1;
    }

    len = len.abs();

    let peeksize = ikcp_peeksize(kcp);

    match peeksize {
        x if x < 0 => return -2,
        x if x > len => return -3,
        _ => {}
    }

    if (kcp.rcv_queue.len() as u32) >= kcp.rcv_wnd {
        recover = true;
    }

    // merge fragment
    len = 0;
    while let Some(seg) = kcp.rcv_queue.front() {
        if let Some(data) = buffer {
            memcpy(data, &seg.data, seg.data.len());
            buffer = Some(&mut data[..seg.data.len()]);
        }

        len += seg.data.len() as i32;
        let fragment = seg.frg as i32;

        if !ispeek {
            kcp.rcv_queue.pop_front();
        }

        if fragment == 0 {
            break;
        }
    }

    // move available data from rcv_buf -> rcv_queue
    while let Some(seg) = kcp.rcv_buf.front() {
        if seg.sn == kcp.rcv_nxt && (kcp.rcv_queue.len() as u32) < kcp.rcv_wnd {
            let x = kcp.rcv_buf.pop_front().unwrap();
            kcp.rcv_queue.push_back(x);
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

// check the size of next message in the recv queue
pub fn ikcp_peeksize(kcp: &IKCPCB) -> i32 {
    let mut length = 0;

    let seg = match kcp.rcv_queue.front() {
        Some(x) => x,
        None => return -1,
    };

    if seg.frg == 0 {
        return seg.data.len() as i32;
    }

    if (kcp.rcv_queue.len() as u32) < (seg.frg + 1) {
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

// user/upper level send, returns below zero for error
pub fn ikcp_send(kcp: &mut IKCPCB, mut buffer: Option<&mut [u8]>, mut len: i32) -> i32 {
    let mut count: i32;

    let mut sent = 0;

    if len < 0 {
        return -1;
    }

    // append to previous segment in streaming mode (if possible)
    if kcp.stream {
        if let Some(old) = kcp.snd_queue.back() {
            if (old.data.len() as u32) < kcp.mss {
                let capacity = (kcp.mss - (old.data.len() as u32)) as i32;
                let extend = len.min(capacity);
                let mut seg = ikcp_segment_new(((old.data.len() as i64) + (extend as i64)) as i32);

                memcpy(&mut seg.data, &old.data, old.data.len());

                if let Some(data) = buffer {
                    memcpy(&mut seg.data[old.data.len()..], data, extend as usize);
                    buffer = Some(&mut data[extend as usize..]);
                }

                seg.frg = 0;
                len -= extend;
                kcp.snd_queue.pop_back();
                kcp.snd_queue.push_back(seg);
                sent = extend;
            }

            if len <= 0 {
                return sent;
            }
        }
    }

    count = match len <= (kcp.mss as i32) {
        true => 1,
        false => (((len as i64) + (kcp.mss as i64) - 1) / (kcp.mss as i64)) as i32,
    };

    if count >= (kcp.rcv_wnd as i32).min(255) {
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
        let size = len.min(kcp.mss as i32);

        let mut seg = ikcp_segment_new(size);

        if let Some(data) = buffer {
            if size > 0 {
                memcpy(&mut seg.data, data, size as usize);
            }

            buffer = Some(&mut data[..size as usize]);
        }

        seg.frg = match !kcp.stream {
            true => (count - i - 1) as u32,
            false => 0,
        };

        kcp.snd_queue.push_back(seg);

        len -= size;
        sent += size;
    }

    sent
}

// parse ack
pub(crate) fn ikcp_update_ack(kcp: &mut IKCPCB, rtt: i32) {
    if kcp.rx_srtt == 0 {
        kcp.rx_srtt = rtt;
        kcp.rx_rttval = rtt / 2;
    } else {
        let mut delta = (rtt - kcp.rx_srtt) as i64;
        delta = delta.abs();
        kcp.rx_rttval = ((((3 * kcp.rx_rttval) as i64) + delta) / 4) as i32;
        kcp.rx_srtt = (7 * kcp.rx_srtt + rtt) / 8;
        kcp.rx_srtt = kcp.rx_srtt.max(1);
    }

    let rto = ((kcp.rx_srtt as i64) + (kcp.interval.max((4 * kcp.rx_rttval) as u32) as i64)) as i32;
    kcp.rx_rto = (rto as u32).clamp(kcp.rx_minrto as u32, IKCP_RTO_MAX) as i32;
}

pub(crate) fn ikcp_shrink_buf(kcp: &mut IKCPCB) {
    kcp.snd_una = match kcp.snd_buf.front() {
        Some(seg) => seg.sn,
        None => kcp.snd_nxt,
    }
}

pub(crate) fn ikcp_parse_ack(kcp: &mut IKCPCB, sn: u32) {
    if ikcp_timediff(sn, kcp.snd_una) < 0 || ikcp_timediff(sn, kcp.snd_nxt) >= 0 {
        return;
    }

    for (i, seg) in kcp.snd_buf.iter().enumerate() {
        if sn == seg.sn {
            kcp.snd_buf.remove(i);
            break;
        }

        if ikcp_timediff(sn, seg.sn) < 0 {
            break;
        }
    }
}

pub(crate) fn ikcp_parse_una(kcp: &mut IKCPCB, una: u32) {
    for (i, seg) in kcp.snd_buf.iter().enumerate() {
        if ikcp_timediff(una, seg.sn) <= 0 {
            kcp.snd_buf.drain(0..i);
            return;
        }
    }

    kcp.snd_buf.clear();
}

pub(crate) fn ikcp_parse_fastack(kcp: &mut IKCPCB, sn: u32, ts: u32) {
    if ikcp_timediff(sn, kcp.snd_una) < 0 || ikcp_timediff(sn, kcp.snd_nxt) >= 0 {
        return;
    }

    for seg in &mut kcp.snd_buf {
        if ikcp_timediff(sn, seg.sn) < 0 {
            break;
        }

        if sn != seg.sn && ikcp_timediff(ts, seg.ts) >= 0 {
            seg.fastack += 1;
        }
    }
}

// ack append
pub(crate) fn ikcp_ack_push(kcp: &mut IKCPCB, sn: u32, ts: u32) {
    kcp.acklist.push((sn, ts));
}

pub(crate) fn ikcp_ack_get(kcp: &IKCPCB, p: i32, sn: &mut u32, ts: &mut u32) {
    (*sn, *ts) = kcp.acklist[p as usize];
}

// parse data
pub(crate) fn ikcp_parse_data(kcp: &mut IKCPCB, newseg: IKCPSEG) {
    let sn = newseg.sn;
    let mut repeat = false;

    if ikcp_timediff(sn, kcp.rcv_nxt + kcp.rcv_wnd) >= 0 || ikcp_timediff(sn, kcp.rcv_nxt) < 0 {
        return;
    }

    let mut index = kcp.rcv_buf.len();
    for (i, seg) in kcp.rcv_buf.iter().enumerate().rev() {
        if seg.sn == sn {
            repeat = true;
            break;
        }

        if ikcp_timediff(sn, seg.sn) > 0 {
            index = i + 1;
            break;
        }
    }

    if !repeat {
        kcp.rcv_buf.insert(index, newseg);
    }

    // move available data from rcv_buf -> rcv_queue
    while let Some(seg) = kcp.rcv_buf.front() {
        if seg.sn == kcp.rcv_nxt && (kcp.rcv_queue.len() as u32) < kcp.rcv_wnd {
            let x = kcp.rcv_buf.pop_front().unwrap();
            kcp.rcv_queue.push_back(x);
            kcp.rcv_nxt += 1;
        } else {
            break;
        }
    }
}

// when you received a low level packet (for example. UDP packet), call it
pub fn ikcp_input(kcp: &mut IKCPCB, mut data: &[u8], mut size: i64) -> i32 {
    let prev_una = kcp.snd_una;

    let mut maxack: u32 = 0;
    let mut latest_ts: u32 = 0;

    let mut flag = false;

    if size < (IKCP_OVERHEAD as i64) {
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

        if size < (IKCP_OVERHEAD as i64) {
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

        if !matches!(
            cmd,
            IKCP_CMD_PUSH | IKCP_CMD_ACK | IKCP_CMD_WASK | IKCP_CMD_WINS
        ) {
            return -3;
        }

        kcp.rmt_wnd = wnd as u32;
        ikcp_parse_una(kcp, una);
        ikcp_shrink_buf(kcp);

        match cmd {
            IKCP_CMD_ACK => {
                if ikcp_timediff(kcp.current, ts) >= 0 {
                    ikcp_update_ack(kcp, ikcp_timediff(kcp.current, ts));
                }

                ikcp_parse_ack(kcp, sn);
                ikcp_shrink_buf(kcp);
                if !flag {
                    flag = true;
                    maxack = sn;
                    latest_ts = ts;
                } else if ikcp_timediff(sn, maxack) > 0 && ikcp_timediff(ts, latest_ts) > 0 {
                    maxack = sn;
                    latest_ts = ts;
                }
            }

            IKCP_CMD_PUSH => {
                if ikcp_timediff(sn, kcp.rcv_nxt + kcp.rcv_wnd) < 0 {
                    ikcp_ack_push(kcp, sn, ts);
                    if ikcp_timediff(sn, kcp.rcv_nxt) >= 0 {
                        seg = ikcp_segment_new(len as i32);
                        seg.conv = conv;
                        seg.cmd = cmd as u32;
                        seg.frg = frg as u32;
                        seg.wnd = wnd as u32;
                        seg.ts = ts;
                        seg.sn = sn;
                        seg.una = una;

                        if len > 0 {
                            memcpy(&mut seg.data, data, len as usize);
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

    if flag {
        ikcp_parse_fastack(kcp, maxack, latest_ts);
    }

    if ikcp_timediff(kcp.snd_una, prev_una) > 0 && kcp.cwnd < kcp.rmt_wnd {
        let mss = kcp.mss;
        if kcp.cwnd < kcp.ssthresh {
            kcp.cwnd += 1;
            kcp.incr += mss;
        } else {
            kcp.incr = kcp.incr.max(mss);
            kcp.incr += (mss * mss) / kcp.incr + (mss / 16);
            if (kcp.cwnd + 1) * mss <= kcp.incr {
                kcp.cwnd = (kcp.incr + mss - 1) / mss.max(1);
            }
        }

        if kcp.cwnd > kcp.rmt_wnd {
            kcp.cwnd = kcp.rmt_wnd;
            kcp.incr = kcp.rmt_wnd * mss;
        }
    }

    0
}

// ikcp_encode_seg
pub(crate) fn ikcp_encode_seg<'a>(mut ptr: &'a mut [u8], seg: &IKCPSEG) -> &'a mut [u8] {
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

pub(crate) fn ikcp_wnd_unused(kcp: &IKCPCB) -> i32 {
    kcp.rcv_wnd.saturating_sub(kcp.rcv_queue.len() as u32) as i32
}

// flush pending data
pub fn ikcp_flush<T, F>(kcp: &mut IKCPCB, buffer: &mut [u8], user: &mut T, output: &mut F)
where
    F: FnMut(&mut [u8], i32, &IKCPCB, &mut T),
{
    let current = kcp.current;
    let position = buffer.as_ptr() as usize;
    let mut ptr = &mut *buffer;

    let mut size: i32;

    let mut cwnd: u32;

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
        if size + (IKCP_OVERHEAD as i32) > (kcp.mtu as i32) {
            ikcp_output(kcp, size, buffer, user, output);
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
        } else if ikcp_timediff(kcp.current, kcp.ts_probe) >= 0 {
            kcp.probe_wait = kcp.probe_wait.max(IKCP_PROBE_INIT);
            kcp.probe_wait += kcp.probe_wait / 2;
            kcp.probe_wait = kcp.probe_wait.min(IKCP_PROBE_LIMIT);
            kcp.ts_probe = kcp.current + kcp.probe_wait;
            kcp.probe |= IKCP_ASK_SEND;
        }
    } else {
        kcp.ts_probe = 0;
        kcp.probe_wait = 0;
    }

    // flush window probing commands
    if (kcp.probe & IKCP_ASK_SEND) != 0 {
        seg.cmd = IKCP_CMD_WASK as u32;
        size = memoffset(ptr, position) as i32;
        if size + (IKCP_OVERHEAD as i32) > (kcp.mtu as i32) {
            ikcp_output(kcp, size, buffer, user, output);
            ptr = &mut *buffer;
        }

        ptr = ikcp_encode_seg(ptr, &seg);
    }

    // flush window probing commands
    if (kcp.probe & IKCP_ASK_TELL) != 0 {
        seg.cmd = IKCP_CMD_WINS as u32;
        size = memoffset(ptr, position) as i32;
        if size + (IKCP_OVERHEAD as i32) > (kcp.mtu as i32) {
            ikcp_output(kcp, size, buffer, user, output);
            ptr = &mut *buffer;
        }

        ptr = ikcp_encode_seg(ptr, &seg);
    }

    kcp.probe = 0;

    // calculate window size
    cwnd = kcp.snd_wnd.min(kcp.rmt_wnd);
    if !kcp.nocwnd {
        cwnd = kcp.cwnd.min(cwnd);
    }

    // move data from snd_queue to snd_buf
    while ikcp_timediff(kcp.snd_nxt, kcp.snd_una + cwnd) < 0 {
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
    let resent = match kcp.fastresend > 0 {
        true => kcp.fastresend as u32,
        false => 0xffffffff,
    };

    let rtomin = match kcp.nodelay == 0 {
        true => (kcp.rx_rto >> 3) as u32,
        false => 0,
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
        } else if ikcp_timediff(current, segment.resendts) >= 0 {
            needsend = true;
            segment.xmit += 1;
            kcp.xmit += 1;
            if kcp.nodelay == 0 {
                segment.rto += segment.rto.max(kcp.rx_rto as u32);
            } else {
                let step = match kcp.nodelay < 2 {
                    true => segment.rto as i32,
                    false => kcp.rx_rto,
                };

                segment.rto += (step / 2) as u32;
            }

            segment.resendts = current + segment.rto;
            lost = true;
        } else if segment.fastack >= resent
            && (segment.xmit <= (kcp.fastlimit as u32) || kcp.fastlimit <= 0)
        {
            needsend = true;
            segment.xmit += 1;
            segment.fastack = 0;
            segment.resendts = current + segment.rto;
            change += 1;
        }

        if needsend {
            segment.ts = current;
            segment.wnd = seg.wnd;
            segment.una = kcp.rcv_nxt;

            size = memoffset(ptr, position) as i32;
            let need = (IKCP_OVERHEAD as i32) + (segment.data.len() as i32);

            if (size + need) > (kcp.mtu as i32) {
                ikcp_output(kcp, size, buffer, user, output);
                ptr = &mut *buffer;

                segment = &mut kcp.snd_buf[i];
            }

            ptr = ikcp_encode_seg(ptr, segment);

            if !segment.data.is_empty() {
                memcpy(ptr, &segment.data, segment.data.len());
                ptr = &mut ptr[segment.data.len()..];
            }

            if segment.xmit >= kcp.dead_link {
                kcp.state = true;
            }
        }
    }

    // flash remain segments
    size = memoffset(ptr, position) as i32;
    if size > 0 {
        ikcp_output(kcp, size, buffer, user, output);
    }

    // update ssthresh
    if change != 0 {
        let inflight = kcp.snd_nxt - kcp.snd_una;
        kcp.ssthresh = inflight / 2;
        kcp.ssthresh = kcp.ssthresh.max(IKCP_THRESH_MIN);
        kcp.cwnd = kcp.ssthresh + resent;
        kcp.incr = kcp.cwnd * kcp.mss;
    }

    if lost {
        kcp.ssthresh = cwnd / 2;
        kcp.ssthresh = kcp.ssthresh.max(IKCP_THRESH_MIN);
        kcp.cwnd = 1;
        kcp.incr = kcp.mss;
    }

    if kcp.cwnd < 1 {
        kcp.cwnd = 1;
        kcp.incr = kcp.mss;
    }
}

// update state (call it repeatedly, every 10ms-100ms), or you can ask
// ikcp_check when to call it again (without ikcp_input/_send calling).
// 'current' - current timestamp in millisec.
pub fn ikcp_update<T, F>(
    kcp: &mut IKCPCB,
    current: u32,
    buffer: &mut [u8],
    user: &mut T,
    output: &mut F,
) where
    F: FnMut(&mut [u8], i32, &IKCPCB, &mut T),
{
    let mut slap: i32;

    kcp.current = current;

    if !kcp.updated {
        kcp.updated = true;
        kcp.ts_flush = kcp.current;
    }

    slap = ikcp_timediff(kcp.current, kcp.ts_flush);

    if !(-10000..10000).contains(&slap) {
        kcp.ts_flush = kcp.current;
        slap = 0;
    }

    if slap >= 0 {
        kcp.ts_flush += kcp.interval;
        if ikcp_timediff(kcp.current, kcp.ts_flush) >= 0 {
            kcp.ts_flush = kcp.current + kcp.interval;
        }

        ikcp_flush(kcp, buffer, user, output);
    }
}

// Determine when should you invoke ikcp_update:
// returns when you should invoke ikcp_update in millisec,
// if there is no ikcp_input/_send calling.
// you can call ikcp_update in that time,
// instead of call update repeatly.
// Important to reduce unnacessary ikcp_update invoking.
// use it to schedule ikcp_update (for example. implementing an epoll-like mechanism,
// or optimize ikcp_update when handling massive kcp connections)
pub fn ikcp_check(kcp: &IKCPCB, current: u32) -> u32 {
    let mut ts_flush = kcp.ts_flush;
    let mut tm_packet = i32::MAX;

    if !kcp.updated {
        return current;
    }

    match ikcp_timediff(current, ts_flush) {
        diff if !(-10000..10000).contains(&diff) => ts_flush = current,
        diff if diff >= 0 => return current,
        _ => {}
    }

    let tm_flush = ikcp_timediff(ts_flush, current);

    for seg in &kcp.snd_buf {
        match ikcp_timediff(seg.resendts, current) {
            diff if diff <= 0 => return current,
            diff if diff < tm_packet => tm_packet = diff,
            _ => {}
        }
    }

    let minimal = (tm_packet.min(tm_flush) as u32).min(kcp.interval);

    current + minimal
}

// change MTU size, default is 1400
pub fn ikcp_setmtu(kcp: &mut IKCPCB, mtu: i32) -> bool {
    if mtu < 50 || mtu < (IKCP_OVERHEAD as i32) {
        return false;
    }

    kcp.mtu = mtu as u32;
    kcp.mss = kcp.mtu - IKCP_OVERHEAD;
    true
}

pub(crate) fn ikcp_interval(kcp: &mut IKCPCB, mut interval: i32) {
    interval = interval.clamp(10, 5000);
    kcp.interval = interval as u32;
}

// fastest: ikcp_nodelay(kcp, 1, 20, 2, 1)
// nodelay: 0: disable(default), 1: enable
// interval: internal update timer interval in millisec, default is 100ms
// resend: 0: disable fast resend(default), 1: enable fast resend
// nc: false: normal congestion control(default), true: disable congestion control
pub fn ikcp_nodelay(kcp: &mut IKCPCB, nodelay: i32, interval: i32, resend: i32, nc: bool) {
    if nodelay >= 0 {
        kcp.nodelay = nodelay as u32;
        kcp.rx_minrto = match nodelay {
            0 => IKCP_RTO_MIN as i32,
            _ => IKCP_RTO_NDL as i32,
        };
    }

    if interval >= 0 {
        ikcp_interval(kcp, interval);
    }

    if resend >= 0 {
        kcp.fastresend = resend;
    }

    kcp.nocwnd = nc;
}

// set maximum window size: sndwnd = 32, rcvwnd = 32 by default
pub fn ikcp_wndsize(kcp: &mut IKCPCB, sndwnd: i32, rcvwnd: i32) {
    if sndwnd > 0 {
        kcp.snd_wnd = sndwnd as u32;
    }

    if rcvwnd > 0 {
        // must >= max fragment size
        kcp.rcv_wnd = (rcvwnd as u32).max(IKCP_WND_RCV);
    }
}

// get how many packet is waiting to be sent
pub fn ikcp_waitsnd(kcp: &IKCPCB) -> i32 {
    (kcp.snd_buf.len() + kcp.snd_queue.len()) as i32
}

// read conv
pub fn ikcp_getconv(kcp: &IKCPCB) -> u32 {
    kcp.conv
}

pub fn ikcp_fastlimit(kcp: &mut IKCPCB, fastlimit: i32) {
    kcp.fastlimit = fastlimit
}

pub fn ikcp_stream(kcp: &mut IKCPCB, stream: bool) {
    kcp.stream = stream
}

pub struct IKCPSEG {
    pub(crate) conv: u32,
    pub(crate) cmd: u32,
    pub(crate) frg: u32,
    pub(crate) wnd: u32,
    pub(crate) ts: u32,
    pub(crate) sn: u32,
    pub(crate) una: u32,
    pub(crate) resendts: u32,
    pub(crate) rto: u32,
    pub(crate) fastack: u32,
    pub(crate) xmit: u32,
    pub(crate) data: Vec<u8>,
}

impl IKCPSEG {
    pub fn conv(&self) -> u32 {
        self.conv
    }
    pub fn cmd(&self) -> u32 {
        self.cmd
    }
    pub fn frg(&self) -> u32 {
        self.frg
    }
    pub fn wnd(&self) -> u32 {
        self.wnd
    }
    pub fn ts(&self) -> u32 {
        self.ts
    }
    pub fn sn(&self) -> u32 {
        self.sn
    }
    pub fn una(&self) -> u32 {
        self.una
    }
    pub fn resendts(&self) -> u32 {
        self.resendts
    }
    pub fn rto(&self) -> u32 {
        self.rto
    }
    pub fn fastack(&self) -> u32 {
        self.fastack
    }
    pub fn xmit(&self) -> u32 {
        self.xmit
    }
    pub fn data(&self) -> &Vec<u8> {
        &self.data
    }
}

pub struct IKCPCB {
    pub(crate) conv: u32,
    pub(crate) mtu: u32,
    pub(crate) mss: u32,
    pub(crate) state: bool,

    pub(crate) snd_una: u32,
    pub(crate) snd_nxt: u32,
    pub(crate) rcv_nxt: u32,

    pub(crate) ssthresh: u32,

    pub(crate) rx_rttval: i32,
    pub(crate) rx_srtt: i32,
    pub(crate) rx_rto: i32,
    pub(crate) rx_minrto: i32,

    pub(crate) snd_wnd: u32,
    pub(crate) rcv_wnd: u32,
    pub(crate) rmt_wnd: u32,
    pub(crate) cwnd: u32,
    pub(crate) probe: u32,

    pub(crate) current: u32,
    pub(crate) interval: u32,
    pub(crate) ts_flush: u32,
    pub(crate) xmit: u32,

    pub(crate) nodelay: u32,
    pub(crate) updated: bool,

    pub(crate) ts_probe: u32,
    pub(crate) probe_wait: u32,

    pub(crate) dead_link: u32,
    pub(crate) incr: u32,

    pub(crate) snd_queue: VecDeque<IKCPSEG>,
    pub(crate) rcv_queue: VecDeque<IKCPSEG>,
    pub(crate) snd_buf: VecDeque<IKCPSEG>,
    pub(crate) rcv_buf: VecDeque<IKCPSEG>,
    pub(crate) acklist: Vec<(u32, u32)>,
    pub(crate) fastresend: i32,
    pub(crate) fastlimit: i32,

    pub(crate) nocwnd: bool,
    pub(crate) stream: bool,
}

impl IKCPCB {
    pub fn conv(&self) -> u32 {
        self.conv
    }
    pub fn mtu(&self) -> u32 {
        self.mtu
    }
    pub fn mss(&self) -> u32 {
        self.mss
    }
    pub fn state(&self) -> bool {
        self.state
    }

    pub fn snd_una(&self) -> u32 {
        self.snd_una
    }
    pub fn snd_nxt(&self) -> u32 {
        self.snd_nxt
    }
    pub fn rcv_nxt(&self) -> u32 {
        self.rcv_nxt
    }

    pub fn ssthresh(&self) -> u32 {
        self.ssthresh
    }

    pub fn rx_rttval(&self) -> i32 {
        self.rx_rttval
    }
    pub fn rx_srtt(&self) -> i32 {
        self.rx_srtt
    }
    pub fn rx_rto(&self) -> i32 {
        self.rx_rto
    }
    pub fn rx_minrto(&self) -> i32 {
        self.rx_minrto
    }

    pub fn snd_wnd(&self) -> u32 {
        self.snd_wnd
    }
    pub fn rcv_wnd(&self) -> u32 {
        self.rcv_wnd
    }
    pub fn rmt_wnd(&self) -> u32 {
        self.rmt_wnd
    }
    pub fn cwnd(&self) -> u32 {
        self.cwnd
    }
    pub fn probe(&self) -> u32 {
        self.probe
    }

    pub fn current(&self) -> u32 {
        self.current
    }
    pub fn interval(&self) -> u32 {
        self.interval
    }
    pub fn ts_flush(&self) -> u32 {
        self.ts_flush
    }
    pub fn xmit(&self) -> u32 {
        self.xmit
    }

    pub fn nodelay(&self) -> u32 {
        self.nodelay
    }
    pub fn updated(&self) -> bool {
        self.updated
    }

    pub fn ts_probe(&self) -> u32 {
        self.ts_probe
    }
    pub fn probe_wait(&self) -> u32 {
        self.probe_wait
    }

    pub fn dead_link(&self) -> u32 {
        self.dead_link
    }
    pub fn incr(&self) -> u32 {
        self.incr
    }

    pub fn snd_queue(&self) -> &VecDeque<IKCPSEG> {
        &self.snd_queue
    }
    pub fn rcv_queue(&self) -> &VecDeque<IKCPSEG> {
        &self.rcv_queue
    }
    pub fn snd_buf(&self) -> &VecDeque<IKCPSEG> {
        &self.snd_buf
    }
    pub fn rcv_buf(&self) -> &VecDeque<IKCPSEG> {
        &self.rcv_buf
    }
    pub fn acklist(&self) -> &Vec<(u32, u32)> {
        &self.acklist
    }
    pub fn fastresend(&self) -> i32 {
        self.fastresend
    }
    pub fn fastlimit(&self) -> i32 {
        self.fastlimit
    }

    pub fn nocwnd(&self) -> bool {
        self.nocwnd
    }
    pub fn stream(&self) -> bool {
        self.stream
    }
}
