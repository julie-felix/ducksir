/* ducksir-client — 客户端那一半（手写，不经打包器；面板只是一个 iframe）。
 *
 * 为什么不做成「把 three.js 打进 DSH 客户端 bundle」：查看器自带 WASM 级的渲染栈和
 * 22 MB 的 GLB，塞进宿主 bundle 既拖慢启动又和宿主的 React 版本耦合。这里只放一个
 * iframe，指向 host 侧那条只读静态路由 `/ducksir-static/client/viewer/index.html`。
 *
 * 加载方式：package.json 里 `dsh.client.platform = "web"` + `exports["./client"]`，
 * 宿主会把它服到 `/plugins/ducksir-client/client.js`。
 */
window.__ModuleLoader__.load({
  id: 'ducksir-client',
  factory: (require) => {
    var module = { exports: {} }
    var exports = module.exports
    var React = require('react')

    var PANEL_ID = 'ducksir-robot'
    var STYLE_ID = 'ducksir-robot-style'
    var VIEWER = '/ducksir-static/client/viewer/index.html'

    var CSS = [
      '.ducksir-robot-panel{display:flex;flex-direction:column;height:100%;min-height:0;background:#0f1216}',
      '.ducksir-robot-bar{display:flex;align-items:center;gap:8px;padding:8px 12px;font-size:12.5px;',
      'color:#c8ccd0;border-bottom:1px solid #262e38;background:#171c22;flex:0 0 auto}',
      '.ducksir-robot-bar .grow{flex:1}',
      '.ducksir-robot-bar a,.ducksir-robot-bar button{color:#8b97a7;background:#1e252d;border:1px solid #262e38;',
      'border-radius:6px;padding:3px 9px;font:inherit;cursor:pointer;text-decoration:none}',
      '.ducksir-robot-bar a:hover,.ducksir-robot-bar button:hover{border-color:#4da3ff;color:#d8dee7}',
      '.ducksir-robot-frame{flex:1;width:100%;border:0;min-height:0;background:#0f1216}',
    ].join('')

    /** 样式只插一次（面板可能被反复挂载）。 */
    function injectStyle() {
      if (typeof document === 'undefined' || document.getElementById(STYLE_ID)) return
      var style = document.createElement('style')
      style.id = STYLE_ID
      style.textContent = CSS
      document.head.appendChild(style)
    }

    /** 侧栏图标：一只方块鸭子。 */
    function Icon() {
      return React.createElement('svg', {
        width: 16, height: 16, viewBox: '0 0 16 16', fill: 'none',
        stroke: 'currentColor', strokeWidth: 1.4, strokeLinecap: 'round', strokeLinejoin: 'round',
        'aria-hidden': 'true',
      },
        React.createElement('path', { d: 'M2.2 6.2h7.6a2 2 0 1 1 0 4H2.2z' }),
        React.createElement('path', { d: 'M9.8 6.2 12.6 4.4v1.8' }),
        React.createElement('path', { d: 'M4.6 10.2v3.2' }),
        React.createElement('path', { d: 'M7.6 10.2v3.2' })
      )
    }

    /** 全屏面板：工具栏 + iframe。 */
    function RobotPanel() {
      var [nonce, setNonce] = React.useState(0)
      var src = VIEWER + '?embed=1&_=' + nonce
      return React.createElement('div', { className: 'ducksir-robot-panel' },
        React.createElement('div', { className: 'ducksir-robot-bar' },
          React.createElement('span', null, '数字描述 · 关节预览'),
          React.createElement('span', { className: 'grow' }),
          React.createElement('a', { href: src, target: '_blank', rel: 'noopener' }, '新标签打开'),
          React.createElement('button', {
            type: 'button',
            onClick: function () { setNonce(function (n) { return n + 1 }) },
          }, '重载')
        ),
        React.createElement('iframe', {
          className: 'ducksir-robot-frame',
          src: src,
          title: '数字描述查看器',
        })
      )
    }

    /** 客户端入口。 */
    function apply(ctx) {
      injectStyle()
      var slots = null
      try {
        slots = typeof ctx.get === 'function' ? ctx.get('slots') : ctx.slots
      } catch (err) {
        slots = null
      }
      if (!slots || typeof slots.inject !== 'function') {
        console.error('[ducksir-client] slots 服务不可用，机器人面板不会注册')
        return
      }
      var disposers = []
      // 侧栏入口
      disposers.push(slots.inject('sidebar.panellist', function () {
        return slots.register(
          { name: 'sidebar.panellist', id: PANEL_ID, order: 31, label: function () { return '机器人' } },
          Icon
        )
      }))
      // 主区面板
      disposers.push(slots.inject('main', function () {
        return slots.register({ name: 'main', key: PANEL_ID }, function () {
          return React.createElement(RobotPanel, null)
        })
      }))
      ctx.effect(function () {
        return function () {
          for (var i = 0; i < disposers.length; i += 1) {
            try { disposers[i]() } catch (err) { /* 卸载时槽位可能已经没了 */ }
          }
        }
      })
    }

    module.exports = { name: 'ducksir-client', inject: ['slots'], apply: apply, default: { name: 'ducksir-client', inject: ['slots'], apply: apply } }
    return module.exports
  }
})
