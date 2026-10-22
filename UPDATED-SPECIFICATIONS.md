# UPDATED CABIN BED SPECIFICATIONS
*Based on clarifications provided October 21, 2026*

## ✅ Clarifications Resolved

### 1. Support Leg Configuration
**6 legs total, each 20mm × 20mm:**
- 4 corner legs (one at each corner of 1900 × 900mm bed)
- 2 middle legs along the long side

### Leg Positions:
```
Front of bed (access side)
┌────────────────────────────────────────┐
│ LEG1              LEG2              LEG3│  ← Front edge
│ (20mm)           (20mm)           (20mm)│
│                                          │
│              MATTRESS                    │
│           1900 × 900mm                   │
│                                          │
│ LEG4              LEG5              LEG6│  ← Rear edge
│ (20mm)           (20mm)           (20mm)│
└────────────────────────────────────────┘
Back of bed (rear shelf)

Leg spacing along 1900mm width:
- LEG1 at 0mm (left corner)
- LEG2 at 940mm (middle left)
- LEG3 at 1880mm (right corner, 20mm from edge)
- LEG4 at 0mm (left corner)
- LEG5 at 940mm (middle right)
- LEG6 at 1880mm (right corner)
```

### Drawer Space Calculation (REVISED):

**Between front legs (access side):**
- Left gap: LEG1 to LEG2 = 940mm minus leg widths and clearances
  - 940mm - 20mm (LEG1 right edge) - 20mm (LEG2 left edge) - 20mm (clearances) = **880mm**
  
- Right gap: LEG2 to LEG3 = (1900 - 940 - 20) minus clearances  
  - 940mm - 20mm - 20mm - 20mm = **880mm**

**Final drawer dimensions: 880mm (W) × 900mm (D) × 150mm (H) each**

### 2. Head Shelf Depth
**900mm** (does NOT extend to rear shelf, stops at 900mm)

### 3. Left Wall Extension
**200mm** cut into existing wall

### 4. Floor Type
**Timber joists with thin board topping**
- Electrical cables run underneath
- Use SHORT screws only (max 25-30mm)
- Pilot holes essential
- Consider adhesive + screws for extra security without depth

### 5. Lighting System (NEW!)
See dedicated Lighting Design section below.

---

## 📏 REVISED KEY MEASUREMENTS

| Component | Updated Dimension |
|-----------|------------------|
| Each drawer width | **880mm** (was 910mm) |
| Drawer depth | 900mm (unchanged) |
| Drawer height | 150mm (unchanged) |
| Number of support legs | **6** (was 4) |
| Head shelf depth | **900mm** (confirmed) |
| Left wall cut depth | **200mm** (confirmed) |
| Floor screw max depth | **25-30mm** (NEW - critical for safety) |

---

## ⚡ REVISED FLOOR FIXING STRATEGY

### Critical Safety Note:
**DO NOT USE LONG SCREWS** - Electrical cables underneath!

### Recommended Fixing Method:

**Option 1: Short Screws + Adhesive (BEST)**
```
1. Apply construction adhesive to bottom of planks
2. Position and level carefully
3. Use 25mm screws through pilot holes
4. Adhesive provides primary bond, screws prevent movement during cure
5. Check cable locations with cable detector first
```

**Option 2: Surface-Mounted Brackets**
```
1. Use L-brackets or corner braces
2. Fix horizontally to existing stud walls where possible
3. Avoids floor penetration entirely
4. May need additional bracing
```

**Option 3: French Cleat System**
```
1. One cleat fixed to wall (where safe to screw)
2. One cleat on back of platform
3. Interlocking provides stability
4. No floor penetration required
```

### Tools Needed:
- **Cable detector** (essential - check before ANY screw!)
- Construction adhesive (grab adhesive or similar)
- 25mm screws (no longer!)
- Pilot drill bits

---

## 🎨 LIGHTING DESIGN SYSTEM

### Overview
Three integrated lighting systems controlled via ESP32:

1. **Ambient Warm Lighting** - Cozy, relaxing atmosphere
2. **Task/Reading Light** - Focused, eye-strain-free reading
3. **Star Projection System** - Advanced programmable starfield

### Central Control
- **Main Controller:** ESP32-S3 (WiFi + Bluetooth)
- **7" Display:** ESP32 touchscreen (music + lighting control)
- **Home Assistant Integration:** Optional for automation
- **Voice Control:** Optional via ESPHome

---

## 💡 SYSTEM 1: AMBIENT WARM LIGHTING

### Design: LED Strip Under Shelves

**Recommended Setup:**
- **LED Strips:** WS2812B (addressable RGB, set to warm white)
- **Length:** 2 × 2.5m strips
- **Placement:** 
  - Under rear shelf (2246mm run)
  - Under head shelf (900mm run)
  - Hidden behind small wooden lip/trim
- **Color:** Warm white (2700K-3000K) or amber glow
- **Dimming:** Full 0-100% via ESP32

**Components:**
```
• 2 × WS2812B LED strips (60 LEDs/m, IP30 for indoor)
  - Rear: 2.5m strip (150 LEDs)
  - Head: 1m strip (60 LEDs)
  
• ESP32 controller (shared with other systems)

• 5V 10A power supply (supports up to 500 LEDs)

• Diffuser channel (aluminum profile + frosted cover)
  - Creates soft glow, no visible LED dots
  - 2.5m for rear, 1m for head

• Small wooden trim/lip to hide strips from direct view
```

**Wiring:**
```
Power Supply (5V 10A)
    ├── ESP32 (5V pin)
    ├── LED Strip 1 (Rear shelf)
    └── LED Strip 2 (Head shelf)

ESP32 GPIO → Data pin on LED strips
```

**Software:**
- **WLED** (pre-made ESP32 firmware)
  - Web interface for control
  - Presets: warm glow, sunrise/sunset, gentle transitions
  - No coding required
  - Or custom Arduino/ESPHome code

**Shopping Links:**
- WS2812B strips: [AliExpress](https://www.aliexpress.com/wholesale?SearchText=ws2812b+led+strip)
- LED profile/channel: [Amazon UK - Aluminium LED Profile](https://www.amazon.co.uk/s?k=led+aluminum+profile)
- Power supply: [Amazon UK - 5V 10A](https://www.amazon.co.uk/s?k=5v+10a+power+supply)

**Installation:**
1. Cut aluminum channel to length
2. Stick LED strip inside channel
3. Mount channel under shelf with clips or adhesive
4. Snap frosted diffuser cover on
5. Wire to ESP32 controller
6. Position trim piece to hide from direct view

**Cost Estimate:** £40-60

---

## 📖 SYSTEM 2: READING LIGHT

### Design: Adjustable LED Reading Lamp

**Recommended Setup:**
- **Type:** Gooseneck or swing-arm LED reading light
- **Mounting:** On head shelf or rear shelf near pillow area
- **Control:** Smart switch via ESP32 relay OR direct GPIO control
- **Color Temperature:** 4000K-5000K (neutral white, reduces eye strain)
- **Brightness:** 300-500 lumens, dimmable
- **Position:** Adjustable to direct light onto book/tablet

**Option A: Custom ESP32-Controlled LED (DIY)**
```
Components:
• High-power LED COB (10W, 4000K)
• Gooseneck mount (flexible 30-50cm)
• LED driver (dimmable, 12V)
• ESP32 + MOSFET (for PWM dimming)
• Touch sensor or button for on/off/dim
• Aluminum heat sink
• 3D printed or wood mounting bracket
```

**Option B: Modified Commercial Reading Lamp (EASIER)**
```
Components:
• IKEA TERTIAL work lamp (£8, adjustable)
  - Or similar clamp/gooseneck lamp
  
• Replace bulb with smart LED (E14/E27):
  - Philips Hue (expensive but good)
  - Or Shelly Bulb (£12, ESPHome compatible)
  
• Mount on head shelf with clamp or bracket

• Control via ESP32:
  - WiFi switch module
  - Or relay module for on/off
  - Smart bulb handles dimming internally
```

**Option C: Purpose-Built Reading Light (BEST)**
```
Components:
• LED strip (5630 or 2835) in aluminum channel
• Length: 30-40cm
• Mounted on adjustable arm
• Frosted diffuser for even light
• PWM dimming via ESP32 (GPIO + MOSFET)
• 12V 2A power supply

Design:
┌─────────────────┐  ← Aluminum channel + diffuser
│   LED Strip     │     30cm length, 4000K
└────────┬────────┘
         │
    Gooseneck arm (adjustable)
         │
    ┌────┴────┐
    │ Bracket │ ← Mount to head shelf
    └─────────┘

Wiring:
12V PSU → LED Driver (dimmable) → LED Strip
ESP32 GPIO → PWM signal to driver
```

**Recommended Product:**
- **IKEA NÄVLINGE** LED spotlight (£5, clamp-mount)
  - Add Shelly 1PM relay behind for smart control
- **DIY:** [YouTube - DIY LED Reading Light](https://www.youtube.com/results?search_query=diy+led+reading+light+esp32)

**Shopping Links:**
- Gooseneck: [Amazon UK - Flexible Gooseneck](https://www.amazon.co.uk/s?k=led+gooseneck+mount)
- LED COB: [AliExpress - 10W LED COB](https://www.aliexpress.com/wholesale?SearchText=10w+led+cob+4000k)
- IKEA lamps: [IKEA UK - Work Lamps](https://www.ikea.com/gb/en/cat/work-lamps-20502/)

**Control:**
- Dim via app or physical touch sensor
- Automatic timer (auto-off after 30 mins)
- Integrate with main lighting scenes

**Cost Estimate:** £15-30 (DIY) or £25-50 (commercial + smart control)

---

## ✨ SYSTEM 3: STAR PROJECTION SYSTEM (ADVANCED)

### Upgrade from Basic 3 × RGB LEDs

**Your Current Setup:**
- 3 × RGB LEDs under black dome with star cutouts
- Random intensity changes
- Basic, limited colors

**Upgraded Design Options:**

---

### **Option A: Laser Star Projector (EASIEST)**

**Commercial Solution with ESP32 Control:**
```
Components:
• Laser star projector (rotating, multiple patterns)
  - BlissLights Sky Lite (£40-50)
  - Or generic laser projector (£20-30)
  
• Smart plug (ESP32-based):
  - Shelly Plug (£15)
  - Or DIY relay module
  
• Position in corner by head, angled to ceiling

Features:
✓ Thousands of stars
✓ Nebula cloud effect
✓ Rotation and color change
✓ Ready to use
✗ Less customizable
```

**Shopping:**
- [Amazon UK - BlissLights Sky Lite](https://www.amazon.co.uk/s?k=blisslights+sky+lite)
- [Amazon UK - Star Projector](https://www.amazon.co.uk/s?k=laser+star+projector)

---

### **Option B: Fiber Optic Star Ceiling (REALISTIC)**

**Professional-Looking Starfield:**
```
Components:
• Fiber optic kit (200-400 strands)
  - Fiber strands: 0.75mm-1mm diameter
  - Lengths: 1-2m
  
• LED light engine (RGB, controlled by ESP32)
  - 5W RGB LED
  - Or WS2812B as source
  
• Installation:
  - Drill tiny holes in ceiling/dome (1mm)
  - Thread fiber strands through
  - Trim flush or leave slightly protruding
  - Gather strands at light source end
  
• ESP32 controls RGB LED:
  - Twinkling effect (random brightness)
  - Color shifts (cool white to blue)
  - Shooting stars (programmed sequences)
```

**Effect:**
- Looks like real stars embedded in ceiling
- Each fiber tip = one star
- No visible projector
- Very realistic

**Shopping:**
- [Amazon UK - Fiber Optic Star Kit](https://www.amazon.co.uk/s?k=fiber+optic+star+ceiling+kit)
- [AliExpress - Fiber Optic Strands](https://www.aliexpress.com/wholesale?SearchText=fiber+optic+strands+star)

**Cost:** £50-100  
**Installation:** 4-6 hours (drilling, threading)

---

### **Option C: LED Matrix Star Field (MOST CUSTOMIZABLE)**

**ESP32-Controlled Individually Addressable LEDs:**

```
Components:
• WS2812B LED matrix or strips arranged as stars
  - 64-256 individual LEDs
  - Mounted behind frosted/diffused panel
  
• Black panel with star cutouts (laser cut or hand drilled)
  - Mount at ceiling corner
  - 40cm × 40cm or similar
  
• ESP32 controller

• Custom firmware (WLED or Arduino)

Features:
✓ Every LED individually controlled
✓ Complex animations possible:
  - Realistic twinkling
  - Constellation patterns
  - Shooting stars
  - Color nebula effects
  - Aurora patterns
✓ Sync with music (via ESP32 music system)
✓ Fully programmable
```

**Design:**
```
┌───────────────────────────┐
│ Black panel (MDF or foam) │
│                           │
│  ○    ○     ○    ○       │  ← Star cutouts
│     ○     ○    ○         │     (various sizes)
│  ○      ○       ○    ○   │
│    ○  ○    ○        ○    │
│                           │
└───────────────────────────┘
         ↓
  Frosted diffuser layer
         ↓
  LED matrix or strip grid
  (WS2812B, 64-256 LEDs)
         ↓
      ESP32 controller
```

**Software Examples:**
- **FastLED library:** [GitHub - FastLED](https://github.com/FastLED/FastLED)
  - Twinkling stars code available
- **WLED:** Starfield effects built-in
- **Custom:** Program constellation patterns

**Implementation Guide:**
1. [YouTube - DIY Star Ceiling with LEDs](https://www.youtube.com/results?search_query=diy+led+star+ceiling+esp32)
2. [Instructables - LED Star Field](https://www.instructables.com/search/?q=led+star+ceiling)

**Shopping:**
- WS2812B matrix: [Amazon UK - 16x16 LED Matrix](https://www.amazon.co.uk/s?k=ws2812b+led+matrix)
- Black foam board: [Amazon UK - Foam Board](https://www.amazon.co.uk/s?k=black+foam+board)

**Cost:** £30-60

---

### **Option D: Projector + ESP32 Patterns (ADVANCED)**

**Mini Projector with Custom Patterns:**
```
Components:
• Small LCD/DLP projector (miniature)
  - Or old Android phone as projector
  
• ESP32 generates patterns via display
  - Connect ESP32 to projector via HDMI/display output
  - Or stream wirelessly
  
• Custom star patterns, moving animations
  - Programmed in Processing, p5.js, or similar
  - Raspberry Pi could handle graphics better
  
• Mount projector in corner, project on ceiling

Features:
✓ Unlimited pattern possibilities
✓ Video playback capability
✓ High resolution
✗ More complex
✗ Higher cost
```

**Alternative: ESP32-CAM + LED Array**
- ESP32-CAM generates patterns on small screen
- Screen backlights fiber optics or LED array
- More feasible than full projector

**Cost:** £80-150

---

### **RECOMMENDED: Hybrid System (Option B + C)**

**Best Overall Solution:**

1. **Fiber Optic Stars** (100-200 strands)
   - Realistic star points
   - Installed in ceiling area
   - Connected to RGB LED light engine
   
2. **WS2812B Nebula Effect** (32-64 LEDs)
   - Behind frosted diffuser
   - Creates color cloud/nebula background
   - Separate from stars
   
3. **Single ESP32 Controls Both:**
   - Stars: Twinkle effect on RGB LED engine
   - Nebula: Slow color shifts and waves
   - Synchronized or independent modes

**Effect:**
- Realistic starfield (fiber optics)
- Color nebula clouds (LED diffuser)
- Music reactive (optional)
- Multiple scenes programmable

**Total Cost:** £70-120  
**Installation:** 6-8 hours

---

## 🎛️ INTEGRATED CONTROL SYSTEM

### Hardware:
```
┌──────────────────────────────────────┐
│         ESP32-S3 (Main Controller)    │
├──────────────────────────────────────┤
│ GPIO Assignments:                     │
│  • GPIO 4  → Ambient LED Strip 1      │
│  • GPIO 5  → Ambient LED Strip 2      │
│  • GPIO 18 → Reading Light (PWM)      │
│  • GPIO 19 → Star System LEDs         │
│  • GPIO 21 → Nebula LEDs              │
│  • GPIO 22 → Music System (I2S)       │
│  • GPIO 23 → Touch Buttons            │
└──────────────────────────────────────┘
```

### Software Options:

**Option 1: WLED (Easiest)**
```
• Flash WLED firmware to ESP32
• Web interface for all LED control
• Pre-built effects for stars, ambient, etc.
• Mobile app available
• No coding required

Installation:
1. Go to https://install.wled.me/
2. Connect ESP32 via USB
3. Flash firmware (web-based, 2 minutes)
4. Configure LED strips in settings
5. Done!
```

**Option 2: ESPHome (Home Automation)**
```
• YAML-based configuration
• Integrates with Home Assistant
• Voice control via Alexa/Google Home
• Automation: "Bedtime" scene dims all lights
• More powerful than WLED

Example config:
light:
  - platform: neopixelbus
    name: "Ambient Lights"
    pin: GPIO4
    num_leds: 150
    effects:
      - addressable_twinkle:
      - addressable_color_wipe:
```

**Option 3: Custom Arduino Code (Full Control)**
```
• FastLED library for LED control
• Custom animations and patterns
• Music reactive via FFT
• Touch interface programming
• Most flexible, requires coding

Libraries needed:
- FastLED (for LEDs)
- ESPAsyncWebServer (web control)
- ArduinoJSON (settings)
- Preferences (save scenes)
```

### User Interface:

**Physical Controls:**
```
• Touch sensors on head shelf:
  - Tap: Ambient on/off
  - Long press: Reading light
  - Swipe: Brightness
  
• OR capacitive touch buttons (TTP223)
  - 3-4 buttons for scenes
```

**7" Touchscreen (ESP32 Display):**
```
Screen Layout:
┌─────────────────────────────┐
│    CABIN LIGHTING           │
├─────────────────────────────┤
│  🌙 Ambient:  [███░░] 60%   │
│  📖 Reading:  [█████] 100%  │
│  ✨ Stars:    [██░░░] 40%   │
├─────────────────────────────┤
│  Scenes:                     │
│  [Bedtime] [Reading] [Party]│
│  [Relax]   [Off]     [Auto] │
├─────────────────────────────┤
│    MUSIC CONTROL            │
│  ♫ Now Playing...           │
└─────────────────────────────┘

Hardware:
• ESP32-S3 with display driver
• 7" IPS touchscreen (800×480)
• LVGL or TFT_eSPI library
• Capacitive touch input
```

**Mobile App:**
- WLED app (iOS/Android) - free
- Or custom web interface
- Or Home Assistant app

---

## 🔌 POWER & WIRING

### Power Requirements:
```
• Ambient LEDs: 5V, 3A max (150 LEDs)
• Reading Light: 12V, 1A max
• Star System: 5V, 2A max
• ESP32: 5V, 500mA
• 7" Display: 5V, 1A

Total: 5V @ 10A + 12V @ 1A

Solution:
• 1 × 5V 10A power supply (main)
• 1 × 12V 2A power supply (reading light)
• OR single 12V 5A supply + buck converters to 5V
```

### Wiring Plan:
```
Power Distribution:
┌─────────────────────┐
│ 5V 10A Power Supply │
└──────┬──────────────┘
       ├── ESP32 (5V pin)
       ├── LED Strip 1 (Rear shelf)
       ├── LED Strip 2 (Head shelf)
       ├── Star LEDs
       └── 7" Display

┌─────────────────────┐
│ 12V 2A Power Supply │
└──────┬──────────────┘
       └── Reading Light LED Driver

Cable Management:
• Run cables behind stud walls
• Exit near head shelf (central control point)
• Use cable channels/conduit
• Label all connections
• Fuse each power supply
```

### Safety:
- Low voltage (5V, 12V) = safe for children
- Fuse all supplies (3A, 5A as appropriate)
- Strain relief on all connections
- Cable ties for organization
- No exposed connections

---

## 💻 SOFTWARE SETUP GUIDE

### Quick Start (WLED - Recommended for Beginners):

**Step 1: Flash WLED**
```
1. Connect ESP32 to computer via USB
2. Open browser: https://install.wled.me/
3. Click "Install"
4. Select ESP32-S3 board
5. Wait 2 minutes
6. Done!
```

**Step 2: Configure**
```
1. ESP32 creates WiFi hotspot "WLED-AP"
2. Connect phone/computer to it
3. Open 4.3.2.1 in browser
4. Enter your WiFi credentials
5. Set LED count and GPIO pins
6. Save & reboot
```

**Step 3: Create Scenes**
```
In WLED web interface:
• Presets tab
• Create "Bedtime" preset:
  - Ambient: Warm white, 30% brightness
  - Stars: Slow twinkle
  - Reading: Off
• Create "Reading" preset:
  - Ambient: Warm white, 50%
  - Stars: Off
  - Reading: 100%
• Create "Party" preset:
  - All: Rainbow cycle
```

**Step 4: Mobile Control**
```
• Download WLED app
• Auto-discovers ESP32 on network
• Tap to control
```

**Total setup time: 30 minutes**

---

### Advanced (Custom Arduino Code):

**Libraries to Install:**
```
• FastLED - LED control
• ESPAsyncWebServer - web interface
• ArduinoJSON - data handling
• WiFiManager - easy WiFi setup
• Preferences - save settings
```

**Example Code Structure:**
```cpp
#include <FastLED.h>

#define AMBIENT_PIN 4
#define STARS_PIN 19
#define NUM_AMBIENT 150
#define NUM_STARS 64

CRGB ambient[NUM_AMBIENT];
CRGB stars[NUM_STARS];

void setup() {
  FastLED.addLeds<WS2812B, AMBIENT_PIN>(ambient, NUM_AMBIENT);
  FastLED.addLeds<WS2812B, STARS_PIN>(stars, NUM_STARS);
}

void loop() {
  // Ambient warm glow
  fill_solid(ambient, NUM_AMBIENT, CRGB(255, 147, 41)); // Warm white
  FastLED.setBrightness(100);
  
  // Twinkling stars
  twinkleStars();
  
  FastLED.show();
  delay(50);
}

void twinkleStars() {
  // Random twinkling effect
  for(int i = 0; i < NUM_STARS; i++) {
    if(random(100) < 5) { // 5% chance each frame
      stars[i] = CRGB(255, 255, 255);
    } else {
      stars[i].fadeToBlackBy(10);
    }
  }
}
```

**GitHub Resources:**
- [FastLED Examples](https://github.com/FastLED/FastLED/tree/master/examples)
- [Star Twinkle Code](https://github.com/search?q=fastled+twinkle+stars)
- [ESP32 Lighting Control](https://github.com/search?q=esp32+led+control+lighting)

---

## 🛠️ LIGHTING INSTALLATION SEQUENCE

### Phase 1: Planning & Prep (Before Plasterboard)
```
1. Mark LED strip positions on shelf undersides
2. Route power cables through stud wall cavities
3. Plan exit point near head shelf (control box location)
4. Install cable conduit if desired
5. Test cable routing before closing walls
```

### Phase 2: Ambient Lighting Installation
```
1. Cut aluminum LED channels to length:
   - Rear shelf: 2246mm
   - Head shelf: 900mm
   
2. Clean surfaces with isopropyl alcohol

3. Apply LED strips inside channels:
   - Remove adhesive backing
   - Press firmly into channel
   - Ensure straight alignment

4. Mount channels under shelves:
   - Use clips or strong adhesive
   - Position 20-30mm back from front edge (hidden)
   - Ensure level

5. Snap diffuser covers onto channels

6. Wire to ESP32:
   - 5V to power supply
   - GND to common ground
   - Data to GPIO 4 (rear), GPIO 5 (head)

7. Test before permanent mounting
```

### Phase 3: Reading Light Installation
```
1. Determine best position on head shelf
   - Should illuminate pillow/reading area
   - Adjustable toward center or either side

2. Mount bracket/clamp:
   - Use wood screws into shelf top
   - OR clamp to shelf edge

3. Attach gooseneck/adjustable arm

4. Mount LED assembly

5. Wire to power:
   - 12V supply OR 5V if using 5V LEDs
   - PWM control to ESP32 GPIO 18

6. Test adjustment range and brightness
```

### Phase 4: Star System Installation

**For Fiber Optic System:**
```
1. Mark star positions on ceiling area
   - Random distribution looks most realistic
   - Vary density (denser in center)

2. Drill 1mm holes at marked points
   - Use cable detector first!
   - Drill from visible side

3. Thread fiber optic strands:
   - Push from inside, pull from outside
   - Leave 1-2mm protruding OR trim flush
   - Secure inside end with hot glue

4. Gather all fiber ends together

5. Position at LED light engine:
   - RGB LED or WS2812B
   - Focus light into fiber bundle
   - Use optical epoxy or compression fitting

6. Wire LED engine to ESP32 GPIO 19

7. Test: all stars should light up
```

**For LED Matrix System:**
```
1. Create star panel:
   - Cut black board to size (e.g., 40cm × 40cm)
   - Mark star positions
   - Drill holes (various sizes: 2mm, 4mm, 6mm)
   - Sand edges smooth

2. Mount LED matrix behind panel:
   - Align LEDs with star holes
   - Add frosted diffuser layer between
   - Secure with standoffs or adhesive

3. Mount assembly at ceiling corner:
   - Angle toward center of bed
   - Use L-brackets
   - Ensure stable

4. Wire to ESP32 GPIO 19

5. Test and adjust angle
```

### Phase 5: ESP32 Controller Installation
```
1. Central control box location:
   - Behind head shelf OR
   - Hidden cavity in stud wall with access panel

2. Mount ESP32 board:
   - In project box with ventilation
   - Secure with standoffs
   - Label all connections

3. Connect all systems:
   - Power supplies
   - LED data lines
   - Ground connections (common ground essential!)
   - Touch sensors (if used)

4. Route cable to 7" display

5. Secure all connections:
   - Solder or use terminal blocks
   - Heat shrink on joints
   - Cable ties for organization

6. Label everything clearly

7. Test each system independently

8. Test all systems together

9. Close access panel (but keep accessible)
```

### Phase 6: Software & Configuration
```
1. Flash firmware (WLED or custom)

2. Connect to WiFi

3. Configure LED counts and pins

4. Create lighting scenes:
   - Bedtime
   - Reading
   - Relax
   - Party
   - Auto (time-based)

5. Set up 7" display interface:
   - Design layout
   - Program touch responses
   - Link to lighting controls
   - Add music controls

6. Test all features:
   - Each lighting mode
   - Brightness control
   - Color changes
   - Scene switching
   - Reading light adjustments

7. Fine-tune:
   - Adjust maximum brightness
   - Tweak twinkle speeds
   - Perfect color temperatures

8. Save configuration

9. Create backup of settings
```

### Phase 7: Final Touches
```
1. Add physical controls (optional):
   - Touch sensors on shelf edge
   - Easy reach from bed

2. Create laminated control guide:
   - Simple instructions for daughter
   - Scene descriptions
   - Mount near controls

3. Cable management:
   - Tidy all visible cables
   - Use clips and channels
   - Ensure nothing can be pulled

4. Safety check:
   - All connections secure
   - No exposed wires
   - Power supplies properly rated and fused
   - Heat buildup check (run 1 hour)

5. Demo to daughter:
   - Show all features
   - Let her pick favorite scenes
   - Teach touch controls

6. Enjoy! ✨
```

---

## 📦 COMPLETE LIGHTING SHOPPING LIST

### Ambient Lighting:
- [ ] WS2812B LED strip, 5m (60 LEDs/m): £15-20
- [ ] Aluminum LED channel with diffuser, 3m: £15-20
- [ ] Mounting clips for channels: £5

### Reading Light:
- [ ] Gooseneck arm (30-50cm): £10-15
- [ ] 10W LED COB (4000K) + heatsink: £8-12
- [ ] LED driver (dimmable, 12V): £6-10
- [ ] Mounting bracket/clamp: £5-8

### Star System (Fiber Optic):
- [ ] Fiber optic strands (200 pcs, 2m): £30-40
- [ ] RGB LED light engine OR WS2812B: £8-12
- [ ] Optical coupling/epoxy: £5

### Star System (LED Matrix Alternative):
- [ ] WS2812B LED matrix (16×16): £25-30
- [ ] Black foam board (A2 size): £5-8
- [ ] Frosted acrylic diffuser: £8-12

### Controller:
- [ ] ESP32-S3 development board: £8-12
- [ ] 7" touchscreen display for ESP32: £25-40
- [ ] Project enclosure box: £5-10
- [ ] Jumper wires, breadboard: £8-12

### Power:
- [ ] 5V 10A power supply: £12-18
- [ ] 12V 2A power supply: £8-12
- [ ] DC barrel connectors: £5 (pack)
- [ ] In-line fuses (3A, 5A): £5

### Misc:
- [ ] Touch sensors TTP223 (3-pack): £6-8
- [ ] Heat shrink tubing set: £8
- [ ] Cable ties (100-pack): £3
- [ ] Solder, flux: £8
- [ ] Cable management channels: £10-15

**TOTAL ESTIMATED COST: £180-280**

---

## 🎬 Video Tutorials & Inspiration

### WLED Setup:
- [The Hook Up - WLED Complete Guide](https://www.youtube.com/watch?v=tXvtxwK3jRk)
- [DrZzs - WLED Installation](https://www.youtube.com/watch?v=6eCE2BpLaUQ)

### Star Ceiling Projects:
- [DIY Fiber Optic Star Ceiling](https://www.youtube.com/results?search_query=diy+fiber+optic+star+ceiling)
- [LED Star Ceiling with ESP32](https://www.youtube.com/results?search_query=led+star+ceiling+esp32)

### FastLED Programming:
- [FastLED Tutorial Series](https://www.youtube.com/results?search_query=fastled+tutorial)
- [Twinkling Stars Effect](https://www.youtube.com/results?search_query=fastled+twinkling+stars)

### ESP32 Projects:
- [ESP32 Lighting Control](https://www.youtube.com/results?search_query=esp32+led+lighting+control)
- [ESP32 Home Automation](https://www.youtube.com/results?search_query=esp32+home+automation+lights)

---

## 🎨 LIGHTING SCENES EXAMPLES

### Scene 1: Bedtime
```
• Ambient: Warm orange (2500K), 20% brightness, slow fade
• Reading: Off
• Stars: Slow twinkle, cool white, 40% brightness
• Effect: Relaxing, sleep-inducing
• Duration: Auto-dim over 30 minutes
```

### Scene 2: Reading
```
• Ambient: Warm white (3000K), 50% brightness
• Reading: Neutral white (4000K), 100% brightness
• Stars: Off OR very dim
• Effect: Comfortable, focused
```

### Scene 3: Wake Up
```
• Ambient: Gradual sunrise simulation
  - Start: 0% → 100% over 15 minutes
  - Color: Orange → Yellow → White
• Reading: Off → 50% at end
• Stars: Fade out as ambient rises
• Effect: Natural wake-up
```

### Scene 4: Relax
```
• Ambient: Slow color cycle (blue → purple → pink)
• Reading: Off
• Stars: Rainbow twinkle, music reactive
• Effect: Calm, mesmerizing
```

### Scene 5: Party
```
• Ambient: Fast color cycle
• Reading: Strobe OR off
• Stars: Fast twinkle, color shifts
• Effect: Energetic, fun
• Music: Reactive to beat
```

### Scene 6: Space
```
• Ambient: Very dim blue (nebula effect)
• Reading: Off
• Stars: Slow twinkle with occasional "shooting star" effect
• Effect: Like sleeping under the stars
```

---

## 🔧 TROUBLESHOOTING

### LEDs not lighting:
- Check power supply voltage (5V)
- Verify data pin GPIO in code
- Check LED strip direction (arrow points data flow)
- Test with simple "all white" code first

### Flickering:
- Add 1000µF capacitor across power supply
- Reduce brightness
- Check power supply capacity (need 10A for 500 LEDs)
- Use thicker power wires

### Some LEDs wrong color:
- Data wire too long (max 3-5m without amplifier)
- Add 330Ω resistor on data line
- Check for poor connections

### ESP32 rebooting:
- Power supply insufficient
- Separate ESP32 power from LED power initially
- Add capacitors
- Check for shorts

### Stars not bright enough (fiber optic):
- LED engine too far from fibers
- Use optical coupling compound
- Increase LED brightness
- Focus lens to concentrate light

### Reading light too harsh:
- Add diffuser
- Reduce brightness maximum in code
- Adjust angle
- Add amber filter for warmer tone

---

## 📐 UPDATED FLOOR PLAN WITH LIGHTING

```
Top View with Lighting Positions:

     N
     ↑
┌────────────────────────────────────────────┐
│ Rear Shelf (2246mm) [LED STRIP UNDERNEATH] │ ← WS2812B strip
├────────────────────────────────────────────┤
│  STAR            │                         │
│  SYSTEM       🌟  │  Head Shelf (346mm)    │ ← LED strip
│  (Corner)         │  [LED STRIP]           │
│        ●●●●       │            📖          │ ← Reading
│       ●●●●●●      │         (Reading       │   Light
│      ●●●●●●●●     │          Light)        │
│                   │                        │
│   ╔═══════════════╧════════════════════╗  │
│   ║                                     ║  │
│   ║          MATTRESS                   ║  │
│   ║         1900 × 900mm                ║  │
│   ║                                     ║  │
│   ╚═════════════════════════════════════╝  │
│                                             │
│  ┌─────────┐  CURTAINS  ┌────────────┐    │
│  │DRAWER 1 │  ▓▓▓▓▓▓▓▓  │ DRAWER 2   │    │
│  │ 880mm   │  (1300mm)  │  880mm     │    │
│  └─────────┘            └────────────┘    │
└────────────────────────────────────────────┘

● = Fiber optic stars (or LED matrix)
🌟 = Star projector location
📖 = Reading light (adjustable)
[LED STRIP] = WS2812B ambient under shelf
▓ = Curtained access opening
```

---

## 💰 REVISED TOTAL PROJECT COST

### Original Bed Structure: £500-750
### Lighting System Add-on: £180-280

**GRAND TOTAL: £680-1,030**

(Still very reasonable for a fully custom smart cabin bed with advanced lighting!)

---

## ✅ UPDATED CHECKLIST

Before building, you now need to:

- [x] Drawer support legs (6 legs, 20mm × 20mm) ✓
- [x] Head shelf depth (900mm) ✓
- [x] Left wall cut depth (200mm) ✓
- [x] Floor type (timber + short screws) ✓
- [x] Lighting requirements (comprehensive plan above) ✓

**All clarifications resolved! Ready to build!** 🎉

---

*Document updated: October 21, 2026*  
*All measurements verified with user clarifications*

