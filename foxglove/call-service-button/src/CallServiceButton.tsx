import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

type State = {
  service: string | null;
};

function CallServiceButton({ context }: { context: PanelExtensionContext }): ReactElement {
  // const [topics, setTopics] = useState<undefined | Immutable<Topic[]>>();
  // const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Build our panel state from the context's initialState, filling in any possibly missing values.
  // const [state, _setState] = useState<State>(() => {
  //   const partialState = context.initialState as Partial<State>;
  //   return {
  //     service: partialState.service ?? null,
  //   };
  // });
  const [state, _setState] = useState<State>({ service: "/brains2/reset" });

  // Respond to actions from the settings editor to update our state.
  const actionHandler = useCallback(
    (action: SettingsTreeAction) => {
      switch (action.action) {
        case "perform-node-action":
          // Handle user-defined actions for nodes in the settings tree
          break;
        case "update":
          if (action.payload.path[0] === "general" && action.payload.path[1] === "Service") {
            // setState({ ...state, service: action.payload.value as string });
            state.service = action.payload.value as string;
          }
          break;
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
            service: {
              label: "Service",
              input: "string",
              value: state.service ?? "bruh",
            },
          },
        },
      },
    });
  }, [context, actionHandler, state]);

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

      // We may have new topics - since we are also watching for messages in the current frame,
      // topics may not have changed It is up to you to determine the correct action when state has
      // not changed.
      // setTopics(renderState.topics);

      // currentFrame has messages on subscribed topics since the last render call
      // if (renderState.currentFrame) {
      //   setMessages(renderState.currentFrame);
      // }
    };

    // After adding a render handler, you must indicate which fields from RenderState will trigger
    // updates. If you do not watch any fields then your panel will never render since the panel
    // context will assume you do not want any updates.

    // tell the panel context that we care about any update to the _topic_ field of RenderState
    // context.watch("topics");

    // tell the panel context we want messages for the current frame for topics we've subscribed to
    // This corresponds to the _currentFrame_ field of render state.
    context.watch("currentFrame");
  }, [context]);

  // invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const handleClick = () => {
    if (!state.service || !context.callService) {
      return;
    }
    // Handle the button click event
    context.callService(state.service, {}).catch((r: unknown) => {
      console.error("Service call failed for reason:", r);
    });
  };

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
      <button className="" onClick={handleClick}>
        Call Service
      </button>
    </div>
  );
}

export function initCallServiceButton(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<CallServiceButton context={context} />);
  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
