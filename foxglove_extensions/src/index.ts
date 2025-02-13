import { ExtensionContext } from "@foxglove/extension";

import { initCallServiceButton } from "./CallServiceButton";
import { initJoystickController } from "./JoystickController";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Call Service Button", initPanel: initCallServiceButton });
  extensionContext.registerPanel({
    name: "Joystick Controller",
    initPanel: initJoystickController,
  });
}
