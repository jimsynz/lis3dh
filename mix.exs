defmodule LIS3DH.MixProject do
  use Mix.Project

  @version "0.1.0"
  def project do
    [
      app: :lis3dh,
      version: @version,
      elixir: "~> 1.19",
      start_permanent: Mix.env() == :prod,
      consolidate_protocols: Mix.env() != :test,
      description: "Driver for the STMicroelectronics LIS3DH 3-axis accelerometer.",
      deps: deps(),
      package: package(),
      docs: [
        main: "readme",
        extras: ["README.md", "CHANGELOG.md"]
      ]
    ]
  end

  def application do
    [
      extra_applications: [:logger]
    ]
  end

  def package do
    [
      maintainers: ["James Harton <james@harton.dev>"],
      licenses: ["Apache-2.0"],
      links: %{
        "Source" => "https://harton.dev/james/lis3dh",
        "GitHub" => "https://github.com/jimsynz/lis3dh"
      }
    ]
  end

  defp deps do
    [
      {:circuits_i2c, "~> 2.0"},
      {:wafer, "~> 1.0"},

      # dev/test
      {:credo, "~> 1.0", only: [:dev, :test], runtime: false},
      {:dialyxir, "~> 1.0", only: [:dev, :test], runtime: false},
      {:doctor, "~> 0.23", only: [:dev, :test], runtime: false},
      {:ex_check, "~> 0.16", only: [:dev, :test], runtime: false},
      {:ex_doc, "~> 0.40", only: [:dev, :test], runtime: false},
      {:git_ops, "~> 2.0", only: [:dev], runtime: false},
      {:igniter, "~> 0.8", only: [:dev, :test]},
      {:mimic, "~> 2.0", only: [:dev, :test]},
      {:mix_audit, "~> 2.0", only: [:dev, :test], runtime: false}
    ]
  end
end
