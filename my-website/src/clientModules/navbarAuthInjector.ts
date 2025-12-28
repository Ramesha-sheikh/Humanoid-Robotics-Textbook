import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

export function onRouteDidUpdate() {
  if (ExecutionEnvironment.canUseDOM) {
    // This will be handled by a navbar wrapper component instead
    // to ensure proper React lifecycle
  }
}
