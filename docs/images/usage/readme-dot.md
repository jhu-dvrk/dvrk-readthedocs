# Usage workflow graph conventions

The `.dot` files in this directory are the sources for the workflow diagrams
in the Usage documentation. Commit both the sources and their generated `.svg`
files. Edit the DOT source rather than the SVG.

## Connections

| Connection | Line | Color | Edge label |
| --- | --- | --- | --- |
| Physical dVRK controller connection | Solid | Red `#dc2626` | `UDP, FireWire` |
| Camera to frame grabber | Solid | Red `#dc2626` | `SDI, HDMI` |
| Computer to physical display | Solid | Red `#dc2626` | `DP, HDMI` |
| Video between processes, including simulator video | Solid | Green `#15803d` | `Stereo`, `Left & right`, or `Stereo overlay` |
| Simulator frontend to worker control/state IPC | Solid | Blue `#2563eb` | `Unix socket IPC` |
| ROS communication | Dashed | Gray `#475569` | `ROS 2 CRTK`, `ROS 2 console status`, or another specific ROS interface |
| Internal component connection | Solid | Gray `#475569` | Describe the commands, state, or events |
| OpenXR application to runtime/headset | Solid | Purple `#7e22ce` | `OpenXR poses, actions` or `OpenXR stereo layers` |

Match edge label colors to line colors. Use arrows in the direction of data
flow; use `dir=both` when commands and state travel in both directions.
Use dashed lines for every ROS connection, including simulator CRTK interfaces.
Internal cisst component interfaces are solid, even when the containing process
also exposes ROS interfaces.

Use commas instead of slashes in all connection labels, including interface
names and lists of signals. Commas in physical connection labels denote
alternative interfaces, not simultaneous connections. Keep details such as the configured `fw`/`udpfw` port,
capture device numbers, and the actual camera interface in the Usage text.
Use red for any additional physical links and label their actual interface.

GStreamer Unix-FD transport shares video buffers through file descriptors
passed over Unix sockets. The legend identifies green edges as GStreamer
video sockets; do not repeat the transport name on each edge or label every
IPC edge as shared memory or GStreamer. The simulator frontend/worker
control channel exchanges JSON over a private Unix stream socket and is blue.
Simulator video remains green, including video sent to a non-simulator process.
When a simulator publishes RTSP instead of a GStreamer socket, label that edge
`RTSP stereo` and use `Video (RTSP, GStreamer socket)` for its green legend
entry. Only draw panel preview connections from publishers that provide a
GStreamer socket; the panel does not consume RTSP directly.

OpenXR runtime interaction is a logical API connection. It does not imply a
DP, HDMI cable: a runtime can deliver headset video through streaming. Draw a
separate red edge only when explicitly showing a physical connection.

## Labels and appearance

Use `package::executable` for ROS package/executable pairs, for example
`dvrk_data::stereo_source`. These identify executables; runtime ROS node names
can differ. Reserve `::` for this relationship in the graphs. The component
label `sawOpenXR / mtsOpenXR` describes a library/component relationship.

- Use the same component name across graphs: `dVRK console`,
  `dvrk_data::stereo_source`, `dvrk_data::stereo_alignment`,
  `dvrk_console::stereo_display`, and `sawOpenXR / mtsOpenXR`.
- Show `dVRK console` as a separate component with the subtitle
  `Inputs and pairing`, including OpenXR examples. Route console input events
  into this component, its enable/clutch/camera signals to teleoperation, and
  its ROS console status to the display. Keep master poses/grippers on a
  separate connection to teleoperation.
- Show `dvrk_console::control_panel` outside the `dvrk_system` cluster with
  the subtitles `Operator controls and status` and `Optional video previews`.
  Use a dashed,
  two-way `ROS 2 commands, status` connection to the console. Also show a
  solid green connection from every GStreamer video publisher in the workflow
  to the panel: raw left/right capture, aligned stereo, simulator stereo, and
  stereo overlay output. Include the green `Stereo overlay` connection from
  `stereo_display` to `control_panel` in every workflow graph; this
  represents consuming the configured overlay socket. Identify only the stream on these
  edges (`Left & right`, `Stereo`, or `Stereo overlay`); omit `GStreamer
  socket(s)` and `Optional preview` because the legend and panel subtitle
  already explain them. These edges represent available sources the panel can
  select, not a requirement to preview all streams simultaneously. Individual
  arm ROS topics are omitted.
- Show a physical `Touch screen` below the control panel, connected from the
  panel by a solid red `DP, HDMI` edge.
- Put the component or executable name first and its role on the next line.
  Keep meaningful differences between physical and simulated setups.
- Use `Stereo` for a single packed stereo stream and `Left & right` for
  the two separate eye sockets. Use `Stereo overlay` for overlaid video.
  Omit socket endpoint names from graphs to keep labels short and layouts
  readable; document the actual endpoints in the Usage text and configuration
  files instead.
- Use top-to-bottom flow (`rankdir=TB`), rounded filled boxes, and a white
  background. Set `fontname="Arial,Helvetica,sans-serif"` on graph, node, and
  edge defaults, including inherited legend text. The generated SVG retains
  this font stack: browsers use Arial, then Helvetica, then their default
  sans-serif font. No font download is needed. Exact metrics vary by system;
  keep labels short and leave padding. For generation on Linux without Arial,
  Liberation Sans provides an Arial-compatible font substitute. Start from an existing graph's defaults.
- Use pale orange `#fff7ed` for robot/camera hardware, pale blue `#eff6ff`
  for control components, pale green `#f0fdf4` for simulator workers,
  pale teal `#f0fdfa` for video components/displays, and pale purple
  `#faf5ff` for OpenXR headsets/runtimes. Connection meaning comes from the
  edge style and label, not the node fill.
- Keep `dvrk_robot::dvrk_system` components in a labeled cluster. Use
  `constraint=false` for cross-links where needed to preserve readable flow.

## Three-column layout

Keep the workflow in three columns, from left to right:

1. **Control:** hardware or OpenXR input, the nested `dvrk_system` cluster,
   and the simulator ROS frontend when present.
2. **Video pipeline:** camera capture or the simulator worker that renders
   video, followed by alignment, overlay, and the physical viewer as applicable.
3. **Operator interface:** `dvrk_console::control_panel` and the touch screen.

Use the column subgraphs and corresponding node `group` attributes from an
existing graph (`cluster_control`/`column_control`, and the matching video and
panel subgraphs). The columns have no visible border or title; invisible
clusters use `label=""; style=invis;`. Keep the nested `dvrk_robot::dvrk_system` cluster visible and labeled.

Align the bottom node of each column in a shared `rank=same`: the last control
component (teleoperation, simulator frontend, or patient cart), the last video
component (HRSV or stereo display), and the touch screen. The invisible
`column_order` node uses `ordering=out` and three invisible outgoing edges in
control/video/panel order to prevent Graphviz from swapping columns. Place
`control_panel` above the touch screen. These layout helpers carry no data.

Use `nodesep=0.4` and `ranksep=1.0` as the starting spacing; centered label
boxes need enough vertical room to avoid one another and the nodes. Regenerate
with `render-dot.py` after label changes; layout is recomputed from source on
each run. Revisit spacing if labels grow or shrink.

Use top-to-bottom edges to order each column, `splines=ortho` for right-angle
routing, and `constraint=false` on cross-column connections so preview inputs
cannot pull the panel below the pipeline or reorder it. Use `xlabel` for all
edge labels: ordinary edge `label` placement is unsupported by orthogonal
routing. Give every connection label a solid white background using a borderless
HTML table with `BGCOLOR="white"` and `CELLPADDING="2"`. Split long labels at
meaningful boundaries with `<BR/>`, typically into two lines; keep short stream
and physical interface names on one line. Escape `&` as `&amp;` in HTML labels.
For example:

```dot
console -> control_panel [
  dir=both, style=dashed, constraint=false,
  xlabel=<<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="2" BGCOLOR="white"><TR><TD>ROS 2<BR/>commands, status</TD></TR></TABLE>>
];
```

`render-dot.py` centers each label on a straight segment of its connection,
preferably near the path midpoint. It moves labels along the connection when
needed to avoid nodes and other labels, and fails if no clear placement exists.
Each label has a solid white rectangular background whose border copies the
connection's color, stroke width, and dash pattern. Labels paint above the
connection strokes. Graphviz alone does not provide this combination reliably
with orthogonal routing, so always use the rendering script for committed SVGs.

Use `minlen=2` on the hardware/headset connection into `dvrk_system` to leave
room for the connection label above the cluster title. White backgrounds must
not obscure component names or the cluster title; inspect the generated SVG.

Keep ROS display status and OpenXR overlay return paths in the gap
between control and video, and panel preview links between video and the panel.
Do not route returns around the operator interface. Compass ports can help, but
check that edges enter a node from outside its boundary. Preserve
the arrows' data direction when changing ranks or ports. OpenXR poses/actions
and stereo layers can share a single purple, double-headed runtime edge.

## Legend

Every graph has a compact legend at the bottom. Copy the `legend_rank`
subgraph from an existing graph and include only the connection types present.
Arrange entries in two columns, read left to right across each row. Use a
20-point spacer between the columns; each entry keeps its sample arrow and
text together. The heading spans all five table cells (two per entry plus the
spacer). Leave the final right-hand entry blank when the count is odd.
Use this order and wording:

1. Red solid: `Physical connection`
2. Green solid: `GStreamer socket (video)`
3. Blue solid: `Unix socket IPC (simulator control/state)`
4. Gray dashed: `ROS 2 communication`
5. Gray solid: `Internal component connection`
6. Purple solid: `OpenXR runtime interaction`

The HTML table uses `━▶` for solid and `┄▶` for dashed samples. Its node has
`shape=plain`, `style=""`, and `margin=0`, with `rank=sink` in its subgraph.
Invisible edges from the workflow's bottom nodes keep it below the workflow;
these are layout aids, not data connections.

For example, a graph with physical, video, and ROS connections can use:

```dot
subgraph legend_rank {
  rank=sink;
  legend [shape=plain, style="", margin=0, label=<
    <TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="3">
      <TR><TD COLSPAN="5" ALIGN="LEFT"><B>Connection legend</B></TD></TR>
      <TR><TD><FONT COLOR="#dc2626">━▶</FONT></TD><TD ALIGN="LEFT"><FONT POINT-SIZE="10">Physical connection</FONT></TD><TD WIDTH="20"></TD><TD><FONT COLOR="#15803d">━▶</FONT></TD><TD ALIGN="LEFT"><FONT POINT-SIZE="10">GStreamer socket (video)</FONT></TD></TR>
      <TR><TD><FONT COLOR="#475569">┄▶</FONT></TD><TD ALIGN="LEFT"><FONT POINT-SIZE="10">ROS 2 communication</FONT></TD><TD WIDTH="20"></TD><TD COLSPAN="2"></TD></TR>
    </TABLE>
  >];
}
display -> legend [style=invis];
```

## Regenerate and check

With Graphviz and Python 3 installed, run from this directory. The renderer
uses only Python's standard library:

```sh
python3 render-dot.py

# Or regenerate one graph:
python3 render-dot.py openxr-real-patient-cart.dot
```

Check Graphviz warnings and inspect each changed SVG for clipped labels,
overlapping edges, a readable legend, and consistency with the Usage text.
The documentation references the SVG files directly, so regenerating them is
required. Avoid renaming the files without updating the corresponding figure
references in `docs/pages/usage/`.
