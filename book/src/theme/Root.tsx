import React, { useEffect } from "react";
import type { Props } from "@theme/Root";

export default function Root({ children }: Props) {
  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const selectedText = selection ? selection.toString().trim() : "";

      const event = new CustomEvent("textSelected", {
        detail: selectedText,
      });

      document.dispatchEvent(event);
    };

    document.addEventListener("mouseup", handleMouseUp);

    return () => {
      document.removeEventListener("mouseup", handleMouseUp);
    };
  }, []);

  return <>{children}</>;
}
