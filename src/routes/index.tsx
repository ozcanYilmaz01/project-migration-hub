import { createFileRoute } from "@tanstack/react-router";

export const Route = createFileRoute("/")({
  head: () => ({
    meta: [
      { title: "PeaceOfMine Operator — dashboard preview" },
      { name: "description", content: "Preview of the PeaceOfMine car operator dashboard with a pretend car." },
      { property: "og:title", content: "PeaceOfMine Operator — dashboard preview" },
      { property: "og:description", content: "Preview of the PeaceOfMine car operator dashboard with a pretend car." },
      { property: "og:type", content: "website" },
      { name: "twitter:card", content: "summary" },
    ],
  }),
  component: Index,
});

function Index() {
  return (
    <iframe
      title="PeaceOfMine operator dashboard (pretend car)"
      src="/dashboard/index.html?mock=1"
      className="fixed inset-0 h-screen w-screen border-0 bg-background"
    />
  );
}
