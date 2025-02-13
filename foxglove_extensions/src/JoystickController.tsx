import { PanelExtensionContext, SettingsTreeAction, Time } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { Joystick } from "react-joystick-component";

type PanelState = {
  topic: string | null;
  enabled: boolean;
  freq: number;
  tau_max: number; // for one wheel
  delta_max: number;
};

type Control = {
  header: {
    frame_id: string;
    stamp: Time;
    seq?: number;
  };
  delta: number;
  tau_fl: number;
  tau_fr: number;
  tau_rl: number;
  tau_rr: number;
};

function now(): Time {
  return { sec: Math.round(Date.now() / 1000), nsec: 0 };
}

function JoystickController({ context }: { context: PanelExtensionContext }): ReactElement {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Build our panel state from the context's initialState, filling in any possibly missing values.
  const [state, setState] = useState<PanelState>(() => {
    const partialState = context.initialState as Partial<PanelState>;
    return {
      topic: partialState.topic ?? null,
      enabled: partialState.enabled ?? true,
      freq: partialState.freq ?? 20,
      tau_max: partialState.tau_max ?? 20.0,
      delta_max: partialState.delta_max ?? 0.3,
    };
  });

  const [control, setControl] = useState<Control>({
    header: { stamp: now(), frame_id: "" },
    delta: 0,
    tau_fl: 0,
    tau_fr: 0,
    tau_rl: 0,
    tau_rr: 0,
  });

  // Respond to actions from the settings editor to update our state.
  const actionHandler = useCallback(
    (action: SettingsTreeAction) => {
      if (action.action === "update") {
        if (action.payload.path[0] === "general") {
          const newState = {
            ...state,
            [action.payload.path[1]!]: action.payload.value,
          };
          if (newState.delta_max < 0) {
            newState.delta_max = 0;
          }
          if (newState.tau_max < 0) {
            newState.tau_max = 0;
          }
          if (newState.freq < 1) {
            newState.freq = 1;
          }
          setState(newState);
        }
      }
    },
    [state],
  );

  // Update the settings editor every time our state or the list of available topics changes.
  useEffect(() => {
    context.saveState(state);
    context.updatePanelSettingsEditor({
      actionHandler,
      nodes: {
        general: {
          label: "General",
          fields: {
            topic: {
              label: "Topic",
              input: "string",
              value: state.topic ?? undefined,
              placeholder: "Enter a valid topic name",
            },
            freq: {
              label: "Frequency",
              input: "number",
              value: state.freq,
              placeholder: "Enter a valid frequency",
              error: state.freq <= 0 ? "Frequency must be greater than 0" : undefined,
            },
            tau_max: {
              label: "Maximum torque per wheel (Nm)",
              input: "number",
              value: state.tau_max,
              placeholder: "Enter a valid torque",
              error: state.tau_max <= 0 ? "Maximum torque must be greater than 0" : undefined,
            },
            delta_max: {
              label: "Maximum steering angle (rad)",
              input: "number",
              value: state.delta_max,
              placeholder: "Enter a valid steering angle",
              error:
                state.delta_max <= 0 ? "Maximum steering angle must be greater than 0" : undefined,
            },
          },
        },
      },
    });
  }, [context, actionHandler, state]);

  // Publish control message
  useEffect(() => {
    if (!context.advertise || !state.topic) {
      return;
    }
    context.advertise(state.topic, "brains2/msg/Controls");
  }, [context, state.topic]);

  const publishControlMessage = useCallback(() => {
    if (!state.topic || !context.publish) {
      return;
    }
    context.publish(state.topic, control);
  }, [context, control, state.topic]);

  // Add a useEffect to handle the periodic publishing
  useEffect(() => {
    if (!state.enabled) {
      return;
    }

    // Calculate interval in milliseconds from frequency in Hz
    const intervalMs = 1000 / state.freq;

    const interval = setInterval(publishControlMessage, intervalMs);

    // Cleanup function to clear the interval when component unmounts
    // or when frequency/enabled state changes
    return () => {
      clearInterval(interval);
    };
  }, [state.freq, state.enabled, publishControlMessage]);

  // We use a layout effect to setup render handling for our panel. We also setup some topic
  // subscriptions.
  useLayoutEffect(() => {
    // The render handler is invoked by Foxglove during playback when your panel needs
    // to render because the fields it is watching have changed. How you handle rendering depends on
    // your framework. You can only setup one render handler - usually early on in setting up your
    // panel.  Without a render handler your panel will never receive updates.  The render handler
    // could be invoked as often as 60hz during playback if fields are changing often.
    context.onRender = (_renderState, done) => {
      // render functions receive a _done_ callback. You MUST call this callback to indicate your
      // panel has finished rendering. Your panel will not receive another render callback until
      // _done_ is called from a prior render. If your panel is not done rendering before the next
      // render call, Foxglove shows a notification to the user that your panel is delayed.  Set the
      // done callback into a state variable to trigger a re-render.
      setRenderDone(() => done);
    };

    // After adding a render handler, you must indicate which fields from RenderState will trigger
    // updates. If you do not watch any fields then your panel will never render since the panel
    // context will assume you do not want any updates.

    // tell the panel context we want messages for the current frame for topics we've subscribed to
    // This corresponds to the _currentFrame_ field of render state.
    context.watch("currentFrame");
  }, [context]);

  // invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div
      style={{
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
        height: "100%",
      }}
    >
      <button
        onClick={() => {
          setState({ ...state, enabled: !state.enabled });
        }}
        style={{
          position: "absolute",
          top: "40px",
          right: "10px",
          padding: "8px 12px",
          border: "none",
          borderRadius: "4px",
          backgroundColor: state.enabled ? "#4CAF50" : "#f44336",
          color: "white",
          cursor: "pointer",
          display: "flex",
          alignItems: "center",
          gap: "4px",
          fontSize: "14px",
        }}
      >
        {state.enabled ? "✅" : "❌"}
      </button>
      <Joystick
        throttle={100}
        size={200}
        stickSize={100}
        sticky={false}
        baseColor={state.enabled ? "gray" : "#e0e0e0"}
        stickColor={state.enabled ? "black" : "#b0b0b0"}
        disabled={!state.enabled}
        move={(e) => {
          if (e.type !== "move") {
            return;
          }
          const delta = -(e.x ?? 0) * state.delta_max;
          const tau = (e.y ?? 0) * state.tau_max;
          setControl({
            header: { stamp: now(), frame_id: "" },
            delta,
            tau_fl: tau,
            tau_fr: tau,
            tau_rl: tau,
            tau_rr: tau,
          });
        }}
        stop={(e) => {
          if (e.type !== "stop") {
            return;
          }
          setControl({
            header: { stamp: now(), frame_id: "" },
            delta: 0.0,
            tau_fl: 0.0,
            tau_fr: 0.0,
            tau_rl: 0.0,
            tau_rr: 0.0,
          });
        }}
      ></Joystick>
      <span
        style={{
          fontSize: "12px",
          color: "gray",
          marginTop: "10px",
        }}
      >
        Steering angle: {control.delta.toFixed(2)} rad
      </span>
      <span
        style={{
          fontSize: "12px",
          color: "gray",
          marginTop: "10px",
        }}
      >
        Wheel torque: {control.tau_fl.toFixed(2)} Nm
      </span>
    </div>
  );
}

export function initJoystickController(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<JoystickController context={context} />);
  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
