module.exports = {
  types: [
    { value: '✨ feat', name: '✨ feat: 添加新功能' },
    { value: '🐛 fix',  name: '🐛 fix: 修复 Bug' },
    { value: '📃 docs', name: '📃 docs: 修改文档' },
    { value: '🎈 perf', name: '🎈 perf: 性能优化' },
    { value: '🧪 test', name: '🧪 test: 添加测试' }
  ],
  scopes: ['fastlio2', 'bspline', 'path_searching', 'plan_env', 'sensor_driver', 'sentry_msgs', 'sentry_bringup'],              // 一定要是空数组，不能省略
  allowCustomScopes: true, // 允许自己输入scope
  messages: {
    type: '选择提交类型:',
    scope: '请输入改动范围（scope，留空无括号）:',
    subject: '请输入简要描述:',
    body: '请输入详细描述（可选）:',
    footer: '关联的 issue（可选）:',
    confirmCommit: '确认提交吗？'
  },
};

