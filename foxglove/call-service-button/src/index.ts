import { ExtensionContext } from "@foxglove/extension";

import { initCallServiceButton } from "./CallServiceButton";
import { initExamplePanel } from "./SettingsPanel";

export function activate(extensionContext: ExtensionContext): void {
  // extensionContext.registerPanel({ name: "example-panel", initPanel: initExamplePanel });
  extensionContext.registerPanel({ name: "Call Service Button", initPanel: initCallServiceButton });
  extensionContext.registerPanel({
    name: "Foxglove Panel Settings Example",
    initPanel: initExamplePanel,
  });
}
