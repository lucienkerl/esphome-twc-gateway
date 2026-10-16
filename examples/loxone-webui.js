// Adds a "Loxone templates" box above the ESPHome web UI: download links for
// GET /loxone/udp.xml and GET /loxone/output.xml (served by the gateway with
// its own IP and the configured UDP port) and the steps to use them. Included via
// web_server: js_include: in loxone-portal.yaml.
const de = (navigator.language || "").toLowerCase().startsWith("de");
const t = de
  ? {
      title: "Loxone: Vorlagen für UDP-Eingang und Ausgang",
      download: "UDP-Eingang (VIU_TWC_Gateway.xml)",
      downloadOut: "Virtueller Ausgang Ca/Tp (VO_TWC_Gateway.xml)",
      steps: [
        "Unten unter „Loxone UDP Port“ einen Port eintragen (z. B. 7000) und „Loxone API Input“ leeren.",
        "Beide Vorlagen herunterladen – IP und Port des Gateways sind schon eingetragen.",
        "Loxone Config: Virtuelle Eingänge → Virtueller UDP Eingang → Vorlage importieren.",
        "„TWC Vc“, „TWC Cac“, „TWC Cp“, „TWC Mr“ mit Vc, Cac, Cp, Mr des Wallbox-Bausteins verbinden; den API-Connector trennen.",
        "Ausgangs-Vorlage unter Virtuelle Ausgänge → Virtueller Ausgang → Vorlage importieren; „TWC Ca“ an Ca, „TWC Tp“ an Tp des Bausteins.",
      ],
    }
  : {
      title: "Loxone: templates for the UDP input and the output",
      download: "UDP input (VIU_TWC_Gateway.xml)",
      downloadOut: "Virtual Output Ca/Tp (VO_TWC_Gateway.xml)",
      steps: [
        "Below, set “Loxone UDP Port” (e.g. 7000) and clear “Loxone API Input”.",
        "Download both templates - the gateway's IP and port are already filled in.",
        "Loxone Config: Virtual Inputs → Virtual UDP Input → import template.",
        "Connect “TWC Vc”, “TWC Cac”, “TWC Cp”, “TWC Mr” to the Wallbox block's Vc, Cac, Cp, Mr; disconnect the API connector.",
        "Import the output template under Virtual Outputs → Virtual Output → import template; connect “TWC Ca” to the block's Ca, “TWC Tp” to its Tp.",
      ],
    };

const box = document.createElement("details");
box.style.cssText =
  "margin:12px auto;max-width:960px;padding:10px 16px;border:1px solid #8884;" +
  "border-radius:8px;font-family:sans-serif;font-size:14px;line-height:1.5";
const summary = document.createElement("summary");
summary.textContent = t.title;
summary.style.cssText = "cursor:pointer;font-weight:600";
box.appendChild(summary);

for (const [href, file, text] of [
  ["/loxone/udp.xml", "VIU_TWC_Gateway.xml", t.download],
  ["/loxone/output.xml", "VO_TWC_Gateway.xml", t.downloadOut],
]) {
  const link = document.createElement("a");
  link.href = href;
  link.download = file;
  link.textContent = text;
  link.style.cssText = "display:block;margin:6px 0;font-weight:600";
  box.appendChild(link);
}

const list = document.createElement("ol");
list.style.cssText = "margin:4px 0 0;padding-left:20px";
for (const step of t.steps) {
  const li = document.createElement("li");
  li.textContent = step;
  list.appendChild(li);
}
box.appendChild(list);

document.body.insertBefore(box, document.body.firstChild);
