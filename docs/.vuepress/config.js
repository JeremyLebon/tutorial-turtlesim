module.exports = {
  title: 'ROS Tutorial - Turtlesim',
  description: 'Awesome description',
  themeConfig: {
    sidebarDepth:5,

    nav: [
      { text: 'Home', link: '/' },
      { text: 'rosdriven', link: 'https://www.rosdriven.dev' },
    ],
    sidebar: [
      ['/', 'Home'],
      ['/installation/', 'Installation'],
      ['/ros-components/', 'ROS components'],
      ['/controlling-the-turtlesim/', 'Controlling the Turtlesim'],
      ['/ros-tools/', 'ROS tools'],
      ['/exercises/', 'Exercises'],
      ['/turtlesim-python/', 'Turtlesim & python']
    ],
    repo: 'https://github.com/JeremyLebon/tutorial-turtlesim',
    docsDir: 'docs',
    docsBranch: 'master'
  },
  markdown: {
    lineNumbers: true,
  },
  serviceWorker: true,
  plugins: [
    ['vuepress-plugin-zooming', {
      // selector for images that you want to be zoomable
      // default: '.content img'
      selector: 'img',

      // make images zoomable with delay after entering a page
      // default: 500
      // delay: 1000,

      // options of zooming
      // default: {}
      options: {
        bgColor: 'black',
        zIndex: 10000,
      },
    }],
      ['vuepress-plugin-code-copy', true],['vuepress-plugin-mermaidjs']
  ],
}
