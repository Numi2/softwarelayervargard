{{/*
Helper template for Vargard Helm chart naming
*/}}
{{- define "vargard.name" -}}
{{- .Chart.Name -}}
{{- end -}}
{{- define "vargard.fullname" -}}
{{- printf "%s-%s" (include "vargard.name" .) .Release.Name -}}
{{- end -}}