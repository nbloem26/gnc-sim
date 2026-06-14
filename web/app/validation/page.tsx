import ValidationFigures from '@/components/ValidationFigures';

export default function ValidationPage() {
  return (
    <>
      <div className="intro">
        <h1>Engineering Validation</h1>
        <p className="muted">
          The interactive simulator is backed by an analytical rigor track (Python).
          These figures demonstrate sensor fidelity and analytical validation of the
          C++ guidance, navigation &amp; control core. Figures are produced by the
          post-processing pipeline; any not yet built render as placeholders.
        </p>
      </div>
      <ValidationFigures />
    </>
  );
}
