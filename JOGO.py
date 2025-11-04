import math, time, threading, random
import pygame
import serial
from serial.tools import list_ports

# ============================ Utils ============================

def clamp(x, a, b):
    return max(a, min(b, x))

class LowPass:
    def __init__(self, a=0.25):
        self.a = a
        self.y = 0.0
        self.init = False
    def update(self, x):
        if not self.init:
            self.y = x
            self.init = True
        else:
            self.y = self.a*x + (1-self.a)*self.y
        return self.y

# ============================ IMU via Bluetooth (SPP) ============================

BT_NAME_HINT = "ESP32_WROOM_IMU"   # dica p/ auto-detecção no Windows
BAUD = 115200

class IMUReader(threading.Thread):
    """
    Lê 'ANG:xx.xx' via porta COM (Bluetooth SPP) e mantém o último ângulo filtrado
    em self.last_angle (graus). Opcionalmente envia 'z' para calibrar zero no firmware.
    """
    def __init__(self, port=None, a=0.25):
        super().__init__(daemon=True)
        self.f = LowPass(a)
        self.last_angle = 0.0
        self.battery_v = None
        self._stop = False
        self.port = port
        self.ser = None

    def stop(self):
        self._stop = True
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except:
            pass

    def close(self):
        self.stop()

    def calibrate_zero(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b"z")
        except:
            pass

    def _find_bt_port(self):
        try:
            ports = list_ports.comports()
            if self.port:
                return self.port
            for p in ports:
                desc = f"{p.description or ''} {p.device or ''}".lower()
                if (BT_NAME_HINT.lower() in desc) or ("bluetooth" in desc) or ("serial" in desc):
                    return p.device
            return ports[0].device if ports else None
        except:
            return None

    def run(self):
        while not self._stop:
            try:
                if not self.ser or not self.ser.is_open:
                    dev = self._find_bt_port()
                    if not dev:
                        time.sleep(1.0)
                        continue
                    self.ser = serial.Serial(dev, BAUD, timeout=1)
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("VBAT:"):
                    try:
                        self.battery_v = float(line.split(":",1)[1])
                    except:
                        pass
                    continue
                if line.startswith("ANG:"):
                    line = line.split(":",1)[1]
                try:
                    ang = float(line)
                    self.last_angle = self.f.update(ang)
                except ValueError:
                    continue
            except Exception:
                try:
                    if self.ser:
                        self.ser.close()
                except:
                    pass
                self.ser = None
                time.sleep(0.8)

# ============================ Mapeamento, unidades e conversões ============================

def map_thr_rev(angle, cfg):
    dz = cfg["deadzone_deg"]
    a = angle
    if abs(a) < dz:
        return 0.0, 0.0
    a_eff = math.copysign(abs(a)-dz, a)
    if a_eff > 0:  # plantar (positivo) -> frente
        t = clamp(a_eff / cfg["angle_max_pl_deg"], 0, 1)
        return t ** cfg["gamma_pos"], 0.0
    else:          # dorsi (negativo) -> ré
        t = clamp((-a_eff) / cfg["angle_max_df_deg"], 0, 1)
        return 0.0, t ** cfg["gamma_neg"]

def kmh_from_pxps(pxps, px_per_m=70.0):
    return (pxps/px_per_m)*3.6

# ====================================== Desenho ======================================

def draw_background(screen, W, H, far_ofs, mid_ofs):
    SKY1, SKY2 = (130,180,255), (210,235,255)
    for y in range(H):
        t = y/max(1, H-1)
        c = (int(SKY1[0]*(1-t)+SKY2[0]*t),
             int(SKY1[1]*(1-t)+SKY2[1]*t),
             int(SKY1[2]*(1-t)+SKY2[2]*t))
        pygame.draw.line(screen, c, (0,y), (W,y))
    ground_y = int(H*0.72)
    pygame.draw.rect(screen, (92,96,105), (0,ground_y, W, H-ground_y))
    HILLS = [(60,160,115), (80,170,125)]
    for i, col in enumerate(HILLS):
        yb = ground_y - 90 - i*18
        path = []
        step = 40
        for x in range(-40, W+80, step):
            y = yb + int(16*math.sin((x - far_ofs*0.2 + i*80)/130.0))
            path.append((x,y))
        path += [(W,ground_y), (0,ground_y)]
        pygame.draw.polygon(screen, col, path)
    random.seed(1)
    for i in range(14):
        x = (i*140 - (mid_ofs*0.5)%140)
        base = ground_y - 10
        pygame.draw.rect(screen, (84,60,40), (x+18, base-30, 6, 30))
        pygame.draw.circle(screen, (60,150,90), (int(x+21), base-44), 16)

def draw_road(screen, W, H, ofs_x):
    road_y = int(H*0.74); road_h = 120
    ROAD = (40,43,48); LANE = (250,220,90)
    pygame.draw.rect(screen, ROAD, (0, road_y, W, road_h))
    seg_w, gap = 80, 50
    x = - (int(ofs_x) % (seg_w + gap))
    y = road_y + road_h//2 - 5
    while x < W:
        pygame.draw.rect(screen, LANE, (x, y, seg_w, 10), border_radius=3)
        x += seg_w + gap

def draw_car(screen, cx, cy, speed_px, fwd_on, rev_on):
    BODY = (30,136,229); BODY_FAST = (16,196,160)
    WINDOW = (230,245,255); WHEEL = (20,20,20)
    REV = (255,255,255); EXH = (210,210,210)
    w, h = 150, 70
    body_col = BODY_FAST if abs(speed_px) > 220 else BODY
    bob = int(2*math.sin(time.time()*8 + speed_px*0.01))
    x, y = cx - w//2, cy - h//2 + bob
    pygame.draw.rect(screen, body_col, (x,y,w,h), border_radius=12)
    pygame.draw.rect(screen, WINDOW, (x+18,y+10,w-36,h-28), border_radius=10)
    wx1, wx2 = x+28, x+w-28; wy = y + h - 6
    pygame.draw.circle(screen, WHEEL, (wx1,wy), 16)
    pygame.draw.circle(screen, WHEEL, (wx2,wy), 16)
    if fwd_on and speed_px > 60:
        for i in range(3):
            r = 5 + i*2
            pygame.draw.circle(screen, EXH, (x-10-i*10, wy-4-i*2), r, 1)
    if rev_on and speed_px < -40:
        pygame.draw.rect(screen, REV, (x+w, y+16, 6, h-32), border_radius=3)

def draw_banner(screen, W, text, color):
    surf = pygame.Surface((W,64), pygame.SRCALPHA)
    pygame.draw.rect(surf, color, (0,0,W,64))
    t = pygame.font.SysFont(None, 42, bold=True).render(text, True, (255,255,255))
    surf.blit(t, (W//2 - t.get_width()//2, 12))
    screen.blit(surf, (0,8))

def draw_phase_bar(screen, W, phase_name, t_left, t_total, color):
    bar = pygame.Surface((W,28), pygame.SRCALPHA)
    pygame.draw.rect(bar, (*color[:3],210), (0,0,W,28))
    p = 1.0 - clamp(t_left/t_total, 0, 1)
    pygame.draw.rect(bar, (255,255,255), (4,4, int((W-8)*p), 20), border_radius=10)
    label = pygame.font.SysFont(None, 24, bold=True).render(f"{phase_name} — {t_left:4.1f}s", True, (18,20,26))
    bar.blit(label, (10,4))
    screen.blit(bar, (0, 74))

def draw_goal_label(screen, W, H, phase, cfg, angle):
    if phase not in ("TRÁS", "FRENTE"):
        return
    if phase == "FRENTE":
        msg = f"Meta FRENTE: ≥ {cfg['target_front']:.0f}°"
        hit = (angle >= cfg["target_front"])
    else:
        msg = f"Meta TRÁS: ≤ {abs(cfg['target_back']):.0f}°"
        hit = (angle <= cfg["target_back"])
    pad_x, pad_y = 16, 8
    font = pygame.font.SysFont(None, 38, bold=True)
    txt = font.render(msg + ("  ✓" if hit else ""), True, (255,255,255))
    w, h = txt.get_width()+2*pad_x, txt.get_height()+2*pad_y
    surf = pygame.Surface((w, h), pygame.SRCALPHA)
    pygame.draw.rect(surf, (0,0,0,210), (0,0,w,h), border_radius=14)
    surf.blit(txt, (pad_x, pad_y))
    screen.blit(surf, (W - w - 28, int(H*0.22)))

def draw_session_info(screen, W, reps_done, reps_total, elapsed_s, vavg_kmh):
    bar = pygame.Surface((W, 24), pygame.SRCALPHA)
    pygame.draw.rect(bar, (0,0,0,140), (0,0,W,24))
    mm = int(elapsed_s//60); ss = int(elapsed_s%60)
    label = pygame.font.SysFont(None, 24).render(f"Reps {reps_done}/{reps_total}  •  Tempo {mm:02d}:{ss:02d}  •  Vmé dia {vavg_kmh:4.1f} km/h", True, (255,255,255))
    bar.blit(label, (12, 2))
    screen.blit(bar, (0, 0))

def draw_hud(screen, W, H, speed_px, dist_signed_px, cfg, angle, ang_rate, state_from_angle, state_from_speed):
    box = pygame.Surface((600, 120), pygame.SRCALPHA)
    pygame.draw.rect(box, (255,255,255,235), (0,0,600,120), border_radius=12)
    kmh = kmh_from_pxps(abs(speed_px), cfg["px_per_m"])
    dir_txt = {"forward":"frente", "reverse":"trás", "neutral":"parado"}[state_from_angle]
    fbig = pygame.font.SysFont(None, 52, bold=True)
    fsm  = pygame.font.SysFont(None, 26)
    box.blit(fbig.render(f"{kmh:4.0f} km/h  (comando: {dir_txt})", True, (18,20,26)), (14, 8))
    box.blit(fsm .render(f"Distância (m): {dist_signed_px/cfg['px_per_m']:7.2f}", True, (60,62,70)), (14, 60))
    real_txt = {"forward":"frente", "reverse":"trás", "neutral":"parado"}[state_from_speed]
    box.blit(fsm .render(f"Movimento: {real_txt}", True, (60,62,70)), (14, 88))
    screen.blit(box, (20,110))
    font1 = pygame.font.SysFont(None, 48, bold=True)
    font2 = pygame.font.SysFont(None, 32)
    col_ang = (0,180,95) if angle>2 else (200,200,200) if angle<-2 else (255,200,40)
    t1 = font1.render(f"Ângulo: {angle:+5.1f}°", True, col_ang)
    var_txt = "Var. + (plantar)" if ang_rate>1.0 else ("Var. − (dorsi)" if ang_rate<-1.0 else "Var. 0")
    col_var = (0,180,95) if ang_rate>1.0 else (200,200,200) if ang_rate<-1.0 else (120,120,120)
    t2 = font2.render(f"{var_txt}  {ang_rate:+5.1f}°/s", True, col_var)
    screen.blit(t1, (W - t1.get_width() - 40, H - 120))
    screen.blit(t2, (W - t2.get_width() - 40, H - 82))

def draw_report(screen, W, H, report, total_time, vavg_kmh, restart_btn_rect):
    overlay = pygame.Surface((W, H), pygame.SRCALPHA)
    overlay.fill((0,0,0,160))
    screen.blit(overlay, (0,0))
    title = pygame.font.SysFont(None, 54, bold=True).render("Sessão concluída", True, (255,255,255))
    screen.blit(title, (W//2 - title.get_width()//2, 40))
    sub = pygame.font.SysFont(None, 32).render(
        f"Tempo total: {int(total_time//60):02d}:{int(total_time%60):02d}  •  Vmé dia: {vavg_kmh:4.1f} km/h",
        True, (230,230,230))
    screen.blit(sub, (W//2 - sub.get_width()//2, 90))
    y = 140
    header = pygame.font.SysFont(None, 28, bold=True).render("Rep  Direção   Ângulo extremo (°)   Tempo p/ Meta (s)", True, (255,255,255))
    screen.blit(header, (80, y))
    y += 12
    pygame.draw.line(screen, (255,255,255), (80,y+14), (W-80,y+14), 1)
    y += 24
    font = pygame.font.SysFont(None, 26)
    for i, r in enumerate(report, start=1):
        t_meta = "—" if (r['t_target'] is None or r['t_target']!=r['t_target']) else f"{r['t_target']:.2f}"
        line = f"{i:>2}    {r['dir']:<7}   {r['ang_ext']:>8.1f}               {t_meta:>8}"
        screen.blit(font.render(line, True, (240,240,240)), (80, y))
        y += 28
    btn_w, btn_h = 260, 56
    x = W//2 - btn_w//2; y = H-100
    restart_btn_rect.x, restart_btn_rect.y, restart_btn_rect.w, restart_btn_rect.h = x, y, btn_w, btn_h
    pygame.draw.rect(screen, (20,120,255), (x,y,btn_w,btn_h), border_radius=12)
    bt = pygame.font.SysFont(None, 36, bold=True).render("RECOMEÇAR  (R)", True, (255,255,255))
    screen.blit(bt, (x + btn_w//2 - bt.get_width()//2, y + btn_h//2 - bt.get_height()//2))

# ====================================== Jogo ======================================

def main():
    pygame.init()
    W, H = 1280, 720
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Rehab Racer — IMU Bluetooth")
    clock = pygame.time.Clock()

    cfg = {
        "deadzone_deg": 2.0,
        "gamma_pos": 0.9, "gamma_neg": 0.9,
        "angle_max_pl_deg": 30.0, "angle_max_df_deg": 30.0,
        "v_max": 980.0, "v_rev_max": 600.0,
        "a_max": 1200.0, "a_rev_max": 900.0,
        "px_per_m": 70.0, "drag": 180.0,
        "start_countdown": 8.0,
        "target_front": +20.0, "target_back": -20.0,
        "angle_enter_deg": 3.0, "angle_exit_deg": 2.0,
        "rep_time": 10.0, "rest_time": 2.0, "reps_each": 5,
        "settle_time": 3.0, "settle_tol_deg": 2.5, "settle_max": 6.0,
    }

    # Protocolo de fases
    seq = []
    for _ in range(cfg["reps_each"]):
        seq += [("TRÁS", cfg["rep_time"]), ("TRANSIÇÃO", cfg["settle_max"]),
                ("FRENTE", cfg["rep_time"]), ("TRANSIÇÃO", cfg["settle_max"])]
    total_reps = cfg["reps_each"]*2

    # ------- IMU Bluetooth SPP -------
    imu = IMUReader(port="COM3")   # <<< TROQUE AQUI para a sua COM Bluetooth de saída (SPP)
    imu.start()

    # Calibração
    zero = 0.0
    calibrating = True
    cal_t0 = time.time()
    cal_buf = []
    CAL_TIME = 4.0

    # Estados & cinematica
    started = False
    start_cd_left = 0.0
    session_done = False
    speed = 0.0
    dist_signed_px = 0.0
    road_x = 0.0; far_ofs = 0.0; mid_ofs = 0.0

    # Fases
    idx = 0
    phase_name, phase_left = seq[0]
    reps_done = 0
    session_t0 = None

    prev_angle_used = 0.0
    angle_state = "neutral"

    # Métricas
    rep_ang_ext = None
    rep_hit = False
    rep_t_to_target = None
    rep_t_elapsed = 0.0
    report_rows = []

    # Transição (zero)
    settle_ok_time = 0.0
    settle_total_time = 0.0

    # Velocidade média absoluta
    acc_abs_dist_m = 0.0
    acc_time_s = 0.0

    restart_btn_rect = pygame.Rect(0,0,0,0)

    def reset_session():
        nonlocal started, start_cd_left, speed, dist_signed_px, road_x, far_ofs, mid_ofs
        nonlocal idx, phase_name, phase_left, reps_done, session_done
        nonlocal prev_angle_used, angle_state
        nonlocal rep_ang_ext, rep_hit, rep_t_to_target, rep_t_elapsed
        nonlocal session_t0, acc_abs_dist_m, acc_time_s
        nonlocal settle_ok_time, settle_total_time, report_rows
        started = True
        start_cd_left = cfg["start_countdown"]
        speed = 0.0; dist_signed_px = 0.0
        road_x = 0.0; far_ofs = 0.0; mid_ofs = 0.0
        idx = 0; phase_name, phase_left = seq[0]
        reps_done = 0; session_done = False
        report_rows.clear()
        prev_angle_used = 0.0; angle_state = "neutral"
        rep_ang_ext = None; rep_hit = False
        rep_t_to_target = None; rep_t_elapsed = 0.0
        session_t0 = time.time()
        acc_abs_dist_m = 0.0; acc_time_s = 0.0
        settle_ok_time = 0.0; settle_total_time = 0.0

    running = True
    while running:
        dt = clock.tick(60) / 1000.0

        # ---------- Eventos ----------
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                elif e.key == pygame.K_SPACE and not started:
                    reset_session()
                elif e.key == pygame.K_r and session_done:
                    reset_session()
                elif e.key == pygame.K_c:
                    calibrating = True
                    cal_t0 = time.time()
                    cal_buf = []
                elif e.key == pygame.K_z:
                    imu.calibrate_zero()
            elif e.type == pygame.MOUSEBUTTONDOWN and session_done:
                if restart_btn_rect.collidepoint(e.pos):
                    reset_session()

        # ---------- Leitura & Calibração ----------
        raw = imu.last_angle
        if calibrating:
            cal_buf.append(raw)
            if time.time() - cal_t0 >= CAL_TIME:
                zero = sum(cal_buf)/max(1,len(cal_buf))
                calibrating = False

        if (not started) or start_cd_left>0 or calibrating or session_done:
            angle = 0.0; ang_rate = 0.0
        else:
            angle = raw - zero
            ang_rate = (angle - prev_angle_used)/max(1e-3, dt)
        prev_angle_used = angle

        # ---------- Estado por histerese ----------
        if not ((not started) or start_cd_left>0 or calibrating or session_done):
            if angle_state == "neutral":
                if angle >= cfg["angle_enter_deg"]:
                    angle_state = "forward"
                elif angle <= -cfg["angle_enter_deg"]:
                    angle_state = "reverse"
            elif angle_state == "forward":
                if angle < cfg["angle_exit_deg"]:
                    angle_state = "neutral"
            elif angle_state == "reverse":
                if angle > -cfg["angle_exit_deg"]:
                    angle_state = "neutral"
        else:
            angle_state = "neutral"

        # ---------- Banners & Fases ----------
        if calibrating:
            banner_text, banner_color = "CALIBRANDO — mantenha o pé parado (1.5 s)", (255,140,0,220)
        elif imu.battery_v is not None and imu.battery_v < 3.6:
            banner_text, banner_color = f"⚠ BATERIA BAIXA: {imu.battery_v:.2f} V — recarregue", (200,60,60,220)
        elif not started:
            banner_text, banner_color = "PRESSIONE ESPAÇO PARA COMEÇAR", (80,120,255,220)
        elif start_cd_left > 0:
            banner_text, banner_color = f"PREPARE-SE: iniciando em {math.ceil(start_cd_left)}s", (50,50,50,220)
            start_cd_left -= dt
        elif session_done:
            banner_text, banner_color = "SESSÃO CONCLUÍDA", (0,140,90,220)
        else:
            if phase_name == "TRÁS":
                banner_text, banner_color = "TRÁS — LEVANTE O PÉ", (180,180,180,220)
            elif phase_name == "FRENTE":
                banner_text, banner_color = "FRENTE — EMPURRE A PONTA DO PÉ", (0,180,95,220)
            elif phase_name == "TRANSIÇÃO":
                banner_text, banner_color = "TRANSIÇÃO — VOLTE AO ZERO", (50,50,50,220)
            else:
                banner_text, banner_color = "DESCANSO — RELAXE", (255,200,40,220)

            if phase_name == "TRANSIÇÃO":
                settle_total_time += dt
                if abs(angle) <= cfg["settle_tol_deg"]:
                    settle_ok_time += dt
                else:
                    settle_ok_time = 0.0
                phase_left -= dt
                if settle_ok_time >= cfg["settle_time"] or settle_total_time >= cfg["settle_max"]:
                    settle_ok_time = 0.0; settle_total_time = 0.0
                    idx += 1
                    if idx >= len(seq):
                        session_done = True
                    else:
                        phase_name, phase_left = seq[idx]
            else:
                phase_left -= dt
                if phase_left <= 0:
                    if phase_name in ("TRÁS","FRENTE"):
                        if phase_name == "FRENTE":
                            rep_ang_ext = max(0.0, rep_ang_ext if rep_ang_ext is not None else 0.0)
                        else:
                            rep_ang_ext = min(0.0, rep_ang_ext if rep_ang_ext is not None else 0.0)
                        report_rows.append({
                            "dir": phase_name,
                            "ang_ext": (rep_ang_ext if rep_ang_ext is not None else float("nan")),
                            "t_target": (rep_t_to_target if rep_t_to_target is not None else float("nan")),
                        })
                        reps_done += 1
                        rep_ang_ext = None; rep_hit = False; rep_t_to_target = None; rep_t_elapsed = 0.0
                    idx += 1
                    if idx >= len(seq):
                        session_done = True
                    else:
                        phase_name, phase_left = seq[idx]

        # ---------- Métricas por repetição ----------
        if started and start_cd_left<=0 and not calibrating and not session_done:
            if phase_name in ("TRÁS","FRENTE"):
                rep_t_elapsed += dt
                if rep_ang_ext is None: rep_ang_ext = angle
                else:
                    rep_ang_ext = max(rep_ang_ext, angle) if phase_name=="FRENTE" else min(rep_ang_ext, angle)
                if not rep_hit:
                    if (phase_name=="FRENTE" and angle>=cfg["target_front"]) or \
                       (phase_name=="TRÁS"   and angle<=cfg["target_back"]):
                        rep_hit = True; rep_t_to_target = rep_t_elapsed

        # ---------- Física ----------
        thr, rev = map_thr_rev(angle, cfg)
        accel = cfg["a_max"]*thr - cfg["a_rev_max"]*rev - cfg["drag"]*(speed/cfg["v_max"])
        if (not started) or (start_cd_left>0) or calibrating or session_done or phase_name=="TRANSIÇÃO":
            accel = 0.0; speed = 0.0
        speed += accel*dt
        speed = clamp(speed, -cfg["v_rev_max"], cfg["v_max"])

        if started and start_cd_left<=0 and not calibrating and not session_done and phase_name!="TRANSIÇÃO":
            road_x += speed * dt
            mid_ofs += speed * dt
            far_ofs += speed * dt * 0.6
            dist_signed_px += speed * dt
            acc_abs_dist_m += abs(speed)*dt / cfg["px_per_m"]
            acc_time_s += dt

        # ---------- Desenho ----------
        screen.fill((0,0,0))
        draw_background(screen, W, H, far_ofs, mid_ofs)
        draw_road(screen, W, H, road_x)
        speed_state = "forward" if speed>8 else ("reverse" if speed<-8 else "neutral")
        draw_car(screen, W//2, int(H*0.70), speed, fwd_on=(speed_state=="forward"), rev_on=(speed_state=="reverse"))
        vavg_kmh = (acc_abs_dist_m/acc_time_s*3.6) if acc_time_s>0 else 0.0
        if session_t0 is not None:
            draw_session_info(screen, W, reps_done, total_reps, (time.time()-session_t0), vavg_kmh)
        draw_banner(screen, W, banner_text, banner_color)
        if started and start_cd_left<=0 and not calibrating and not session_done:
            if phase_name in ("TRÁS", "FRENTE"):
                total = cfg["rep_time"]
                color = (180,180,180) if phase_name=="TRÁS" else (0,180,95)
                draw_phase_bar(screen, W, phase_name, phase_left, total, color)
            elif phase_name == "TRANSIÇÃO":
                total = max(cfg["settle_time"], 0.01)
                temp_left = max(0.0, total - settle_ok_time)
                draw_phase_bar(screen, W, "TRANSIÇÃO (no zero)", temp_left, total, (50,50,50))
            draw_goal_label(screen, W, H, phase_name, cfg, angle)
        draw_hud(screen, W, H, speed, dist_signed_px, cfg, angle, ang_rate, angle_state, speed_state)

        if session_done:
            total_time = (time.time()-session_t0) if session_t0 else 0.0
            draw_report(screen, W, H, report_rows, total_time, vavg_kmh, restart_btn_rect)

        pygame.display.flip()

    imu.stop()
    pygame.quit()

if __name__ == "__main__":
    main()
