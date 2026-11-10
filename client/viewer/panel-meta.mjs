// 面板 8 项的中文标签与单位——和 sim/host/panel.py 的 PANEL_KEYS 一一对应。
// 改这里就要改那边（有 test_panel.py 守 schema）。
export const PANEL_META = {
  control_hz: { label: '控制频率', unit: 'Hz' },
  physics_hz: { label: '物理频率', unit: 'Hz' },
  realtime_factor: { label: '实时倍率', unit: '×' },
  control_p95_ms: { label: '控制步 p95', unit: 'ms' },
  deadline_miss: { label: '超期步数', unit: '步' },
  base_height_min: { label: '基座最低', unit: 'm' },
  upright_final: { label: '末态直立度', unit: '' },
  distance: { label: '水平位移', unit: 'm' },
};
