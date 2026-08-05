---@module 'hl'

-- Monitors ##########

require("monitors")

-- ENVIRONMENT #########

hl.env("XDG_MENU_PREFIX", "plasma-")

-- VARIABLES #########

hl.config({
    cursor = {
        no_warps = true,
    },
})

hl.config({
    debug = {
        disable_logs = false,
        vfr = true,
    },
})

hl.config({
    ecosystem = {
        no_update_news = true,
        no_donation_nag = true,
    },
})

hl.config({
    input = {
        kb_layout = "pl",
        -- kb_options = grp:ctrls_toggle
        repeat_rate = 20,
        repeat_delay = 350,
        touchpad = {
            disable_while_typing = 0,
            natural_scroll = 1,
            clickfinger_behavior = 0,
            middle_button_emulation = 0,
            tap_to_click = 1,
        },
        sensitivity = 0.0,
        -- -1.0 - 1.0, 0 means no modification.
        left_handed = false,
    },
})

hl.config({
    gestures = {
        -- workspace_swipe=true 
        workspace_swipe_min_speed_to_force = 5,
    },
})

hl.config({
    group = {
        drag_into_group = 2,
        merge_floated_into_tiled_on_groupbar = true,
        groupbar = {
            height = 20,
            indicator_height = 3,
            font_size = 12,
            gradients = true,
            scrolling = false,
            text_color = "rgb(ffffff)",
            gaps_in = 0,
            gaps_out = 0,
        },
    },
})

hl.config({
    general = {
        gaps_in = 2,
        gaps_out = 3,
        border_size = 1,
        resize_on_border = true,
        layout = "dwindle",
    },
})

hl.config({
    decoration = {
        -- See https://wiki.hyprland.org/Configuring/Variables/ for more
        rounding = 5,
        blur = {
            enabled = true,
            size = 3,
            passes = 1,
            new_optimizations = true,
        },
        shadow = {
            enabled = true,
            range = 4,
            render_power = 3,
            color = "rgba(1a1a1aee)",
        },
    },
})

-- Blur for waybar 

-- TODO: manual review: blurls = "waybar"

hl.config({
    animations = {
        enabled = true,
        -- Some default animations, see https://wiki.hyprland.org/Configuring/Animations/ for more
    },
})

hl.config({
    dwindle = {
        -- See https://wiki.hyprland.org/Configuring/Dwindle-Layout/ for more
        preserve_split = true,
        -- you probably want this
    },
})

hl.config({
    master = {
        -- See https://wiki.hyprland.org/Configuring/Master-Layout/ for more
        new_status = "master",
    },
})

hl.config({
    misc = {
        disable_hyprland_logo = true,
        disable_splash_rendering = true,
        mouse_move_enables_dpms = true,
        focus_on_activate = true,
    },
})

-- GESTURES #########

hl.gesture({
    fingers = 3,
    direction = "horizontal",
    action = "workspace",
})

-- BINDS #################

local mainMod = "SUPER"

hl.bind(mainMod .. " + " .. "Return", hl.dsp.exec_cmd("konsole --hide-menubar"))
hl.bind(mainMod .. " + " .. "C", hl.dsp.window.close())
hl.bind(mainMod .. " + " .. "F", hl.dsp.window.fullscreen({ mode = "maximized" }))
hl.bind(mainMod .. " + " .. "space", hl.dsp.window.float())
hl.bind(mainMod .. " + " .. "P", hl.dsp.window.pseudo())
hl.bind(mainMod .. " + " .. "E", hl.dsp.layout("togglesplit"))
hl.bind(mainMod .. " + " .. "D", hl.dsp.exec_cmd("rofi -show drun"))
hl.bind(mainMod .. " + " .. "W", hl.dsp.group.toggle())
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "Return", hl.dsp.exec_cmd("dolphin"))
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "Q", hl.dsp.window.close())
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "E", hl.dsp.exit())
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "F", hl.dsp.window.fullscreen({ mode = "fullscreen" }))
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "space", hl.dsp.window.pin())

hl.bind("XF86AudioMute", hl.dsp.exec_cmd("~/.local/share/hypr/scripts/volume.sh mute"))
hl.bind("XF86AudioLowerVolume", hl.dsp.exec_cmd("~/.local/share/hypr/scripts/volume.sh down"))
hl.bind("XF86AudioRaiseVolume", hl.dsp.exec_cmd("~/.local/share/hypr/scripts/volume.sh up"))
hl.bind("XF86AudioMicMute", hl.dsp.exec_cmd("pactl set-source-mute @DEFAULT_SOURCE@ toggle"))
hl.bind("XF86MonBrightnessDown", hl.dsp.exec_cmd("~/.local/share/hypr/scripts/brightness.sh down"))
hl.bind("XF86MonBrightnessUp", hl.dsp.exec_cmd("~/.local/share/hypr/scripts/brightness.sh up"))

hl.bind(mainMod .. " + " .. "PRINT", hl.dsp.exec_cmd("hyprshot -m region -o ~/Pictures/screenshots"))
hl.bind("PRINT", hl.dsp.exec_cmd("hyprshot -m output -o ~/Pictures/screenshots"))
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "PRINT", hl.dsp.exec_cmd("hyprshot -m window -o ~/Pictures/screenshots"))

hl.bind(mainMod .. " + " .. "L", hl.dsp.exec_cmd("hyprlock"))

-- Move focus with mainMod + arrow keys

hl.bind(mainMod .. " + " .. "left", hl.dsp.focus({ direction = "left" }))
hl.bind(mainMod .. " + " .. "right", hl.dsp.focus({ direction = "right" }))
hl.bind(mainMod .. " + " .. "up", hl.dsp.focus({ direction = "up" }))
hl.bind(mainMod .. " + " .. "down", hl.dsp.focus({ direction = "down" }))

-- Move focus in a group

hl.bind(mainMod .. " + " .. "semicolon", hl.dsp.group.next({ forward = false }))
hl.bind(mainMod .. " + " .. "apostrophe", hl.dsp.group.next())
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "semicolon", hl.dsp.group.move_window({ forward = false }))
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "apostrophe", hl.dsp.group.move_window({ forward = true }))

-- Switch workspaces with mainMod + [0-9]

local workspaceKeyMap = {}

for workspace = 1, 9 do
    workspaceKeyMap[#workspaceKeyMap + 1] = {
        key = tostring(workspace),
        workspace = workspace,
    }
end

workspaceKeyMap[#workspaceKeyMap + 1] = {
    key = "0",
    workspace = 10,
}

for _, item in ipairs(workspaceKeyMap) do
    hl.bind(mainMod .. " + " .. item.key, hl.dsp.focus({ workspace = item.workspace }))
end

-- Scroll through existing workspaces with mainMod + scroll or mainMod + [ or ]

hl.bind(mainMod .. " + " .. "mouse_down", hl.dsp.focus({ workspace = "m-1" }))
hl.bind(mainMod .. " + " .. "mouse_up", hl.dsp.focus({ workspace = "m+1" }))
hl.bind(mainMod .. " + " .. "bracketleft", hl.dsp.focus({ workspace = "m-1" }))
hl.bind(mainMod .. " + " .. "bracketright", hl.dsp.focus({ workspace = "m+1" }))

-- Move active window to a workspace with mainMod + SHIFT + [0-9]

for _, item in ipairs(workspaceKeyMap) do
    hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. item.key, hl.dsp.window.move({ workspace = item.workspace }))
end

-- Move active window in a direction

hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "left", hl.dsp.window.move({ direction = "left" }))
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "right", hl.dsp.window.move({ direction = "right" }))

-- Move active workspace to a monitor

hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "CTRL" .. " + " .. "left", hl.dsp.workspace.move({ monitor = "l" }))
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "CTRL" .. " + " .. "right", hl.dsp.workspace.move({ monitor = "r" }))

-- Move/resize windows with mainMod + LMB/RMB and dragging

hl.bind(mainMod .. " + " .. "mouse:272", hl.dsp.window.drag(), { mouse = true })
hl.bind(mainMod .. " + " .. "CTRL" .. " + " .. "mouse:272", hl.dsp.window.resize(), { mouse = true })
hl.bind(mainMod .. " + " .. "mouse:273", hl.dsp.window.resize(), { mouse = true })

-- Pyprland

hl.bind(mainMod .. " + " .. "V", hl.dsp.exec_cmd("pypr toggle volume"))
hl.bind(mainMod .. " + " .. "grave", hl.dsp.exec_cmd("pypr toggle term"))
hl.bind(mainMod .. " + " .. "A", hl.dsp.exec_cmd("pypr fetch_client_menu"))
hl.bind(mainMod .. " + " .. "SHIFT" .. " + " .. "A", hl.dsp.exec_cmd("pypr unfetch_client"))
hl.bind(mainMod .. " + " .. "Z", hl.dsp.exec_cmd("pypr zoom"))

-- WINDOW RULES ###########

hl.window_rule({
    name  = "float_common_kde_apps",
    match = {
        class = "^(org.kde.konsole|org.kde.kcalc)$",
    },
    float = true,
})

hl.window_rule({
    name  = "pip_compact_pinned",
    match = {
        title = "^(Picture in picture|Picture-in-Picture)$",
    },
    float = true,
    pin = true,
    rounding = 0,
    border_size = 0,
    persistent_size = true,
})

-- REAPER hosts EZdrummer (XWayland), so target popups by REAPER class/title.

hl.window_rule({
    name  = "reaper_ezdrummer",
    match = {
        class = "^(REAPER)$",
        title = "^(FX:.*EZdrummer.*)$",
    },
    no_anim = true,
    focus_on_activate = true,
})

hl.window_rule({
    name  = "reaper_xwayland_allows_input",
    match = {
        class = "^(REAPER)$",
        xwayland = 1,
    },
    allows_input = true,
})

hl.window_rule({
    name  = "reaper_empty_title_xwayland",
    match = {
        class = "^(REAPER)$",
        title = "^$",
        xwayland = 1,
    },
    stay_focused = true,
    focus_on_activate = true,
    no_anim = true,
    suppress_event = "activate activatefocus",
})

hl.window_rule({
    name  = "reaper_modal_xwayland",
    match = {
        class = "^(REAPER)$",
        modal = 1,
        xwayland = 1,
    },
    stay_focused = true,
    focus_on_activate = true,
    suppress_event = "activate activatefocus",
})

-- Smart gaps

hl.workspace_rule({
    workspace = "w[tv1]",
    gaps_out = 0,
    gaps_in = 0,
})

hl.workspace_rule({
    workspace = "f[1]",
    gaps_out = 0,
    gaps_in = 0,
})

hl.window_rule({
    name  = "smart_gaps_tv1",
    match = {
        float = 0,
        workspace = "w[tv1]",
    },
    border_size = 0,
    rounding = 0,
})

hl.window_rule({
    name  = "smart_gaps_fullscreen_layout",
    match = {
        float = 0,
        workspace = "f[1]",
    },
    border_size = 0,
    rounding = 0,
})

hl.config({
    xwayland = {
        force_zero_scaling = true,
    },
})

-- Autostart
hl.on("hyprland.start", function()
    hl.exec_cmd("pypr")
end)
