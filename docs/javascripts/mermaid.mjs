import mermaid from "https://unpkg.com/mermaid@11/dist/mermaid.esm.min.mjs";

document.querySelectorAll("pre.mermaid > code").forEach((code) => {
  const pre = code.parentElement;
  const diagram = document.createElement("div");
  diagram.className = pre.className;
  diagram.textContent = code.textContent;
  pre.replaceWith(diagram);
});

mermaid.initialize({ startOnLoad: true });

const mermaidSelector = ".mermaid";

function createDiagramDialog() {
  const dialog = document.createElement("dialog");
  dialog.className = "mermaid-dialog";
  dialog.setAttribute("aria-label", "Enlarged Mermaid diagram");
  dialog.innerHTML = `
    <form method="dialog" class="mermaid-dialog__header">
      <button class="mermaid-dialog__close" aria-label="Close diagram">×</button>
    </form>
    <div class="mermaid-dialog__content"></div>
  `;
  dialog.addEventListener("click", (event) => {
    if (event.target === dialog) dialog.close();
  });
  document.body.append(dialog);
  return dialog;
}

function enhanceMermaidDiagrams() {
  document.querySelectorAll(mermaidSelector).forEach((diagram) => {
    if (diagram.dataset.zoomBound || !diagram.querySelector("svg")) return;

    diagram.dataset.zoomBound = "true";
    const button = document.createElement("button");
    button.className = "mermaid-open-button";
    button.type = "button";
    button.title = "Open diagram in a larger window";
    button.setAttribute("aria-label", "Open diagram in a larger window");
    button.innerHTML = '<span aria-hidden="true">🔍</span><span>Open diagram</span>';
    button.addEventListener("click", () => {
      const dialog = document.querySelector(".mermaid-dialog") || createDiagramDialog();
      dialog.querySelector(".mermaid-dialog__content").replaceChildren(
        diagram.querySelector("svg").cloneNode(true),
      );
      dialog.showModal();
    });
    diagram.after(button);
  });
}

const observer = new MutationObserver(enhanceMermaidDiagrams);
observer.observe(document.body, { childList: true, subtree: true });
enhanceMermaidDiagrams();
