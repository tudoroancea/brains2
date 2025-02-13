import { ExtensionContext } from "@foxglove/extension";

import { initCallServiceButton } from "./CallServiceButton";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Call Service Button", initPanel: initCallServiceButton });
}
